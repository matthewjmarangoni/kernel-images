package e2e

// FFmpeg compatibility suite.
//
// The images ship a pinned static ffmpeg build (FFMPEG_VERSION in each image's
// Dockerfile). These tests exist so that bumping that pin fails CI instead of
// production: every check drives the real server (recorder and screenshot
// APIs) so the server's own ffmpeg arguments run against the image's own
// binary, then inspects the result with that image's ffprobe.
//
// Each image gets one container; subtests share it and run sequentially so
// they don't compete for the X display or CPU.

import (
	"bytes"
	"context"
	"encoding/base64"
	"encoding/binary"
	"encoding/json"
	"image"
	_ "image/jpeg"
	"image/png"
	"net/http"
	"os/exec"
	"regexp"
	"slices"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/kernel/kernel-images/server/lib/cdpmonitor"
	instanceoapi "github.com/kernel/kernel-images/server/lib/oapi"
	"github.com/stretchr/testify/require"
)

const (
	ffmpegTestWidth  = 1280
	ffmpegTestHeight = 720
	// Matches FRAME_RATE in the images' kernel-images-api supervisor config.
	ffmpegDefaultFrameRate = 10
	// Stop sends SIGINT and only escalates to SIGTERM after a minute. A stop
	// that takes anywhere near that long means ffmpeg stopped honouring SIGINT.
	ffmpegMaxGracefulStop = 15 * time.Second
)

func TestFFmpegCompatibility(t *testing.T) {
	if _, err := exec.LookPath("docker"); err != nil {
		t.Skipf("docker not available: %v", err)
	}

	images := []struct {
		name  string
		image string
		// headless Chromium never paints to the X display, so only headful
		// can produce on-screen motion.
		rendersToDisplay bool
	}{
		{name: "headless", image: headlessImage},
		{name: "headful", image: headfulImage, rendersToDisplay: true},
	}

	for _, img := range images {
		t.Run(img.name, func(t *testing.T) {
			t.Parallel()

			ctx, cancel := context.WithTimeout(context.Background(), 15*time.Minute)
			defer cancel()

			c := NewTestContainer(t, img.image)
			require.NoError(t, c.Start(ctx, ContainerConfig{
				Env: map[string]string{
					"WIDTH":  strconv.Itoa(ffmpegTestWidth),
					"HEIGHT": strconv.Itoa(ffmpegTestHeight),
				},
			}), "failed to start container")
			defer c.Stop(ctx)
			defer dumpAPILogOnFailure(t, c)
			require.NoError(t, c.WaitReady(ctx), "api not ready")

			// WIDTH/HEIGHT only configure Xvfb; headful Xorg needs the API.
			if w, h, err := getXRootResolution(ctx, c); err != nil || w != ffmpegTestWidth || h != ffmpegTestHeight {
				patchDisplayExpectingOK(t, ctx, c, ffmpegTestWidth, ffmpegTestHeight, 60)
			}
			waitForXRootResolution(t, ctx, c, ffmpegTestWidth, ffmpegTestHeight, 30*time.Second)

			client, err := c.APIClient()
			require.NoError(t, err, "failed to create API client")

			t.Run("Capabilities", func(t *testing.T) {
				testFFmpegCapabilities(t, ctx, c)
			})
			t.Run("Recording", func(t *testing.T) {
				testFFmpegRecording(t, ctx, c, client)
			})
			t.Run("RecordingExplicitFrameRate", func(t *testing.T) {
				testFFmpegRecordingExplicitFrameRate(t, ctx, c, client)
			})
			t.Run("RecordingAudio", func(t *testing.T) {
				testFFmpegRecordingAudio(t, ctx, c, client)
			})
			t.Run("RecordingMaxDuration", func(t *testing.T) {
				testFFmpegRecordingMaxDuration(t, ctx, c, client)
			})
			t.Run("RecordingChapters", func(t *testing.T) {
				testFFmpegRecordingChapters(t, ctx, c, client)
			})
			t.Run("Screenshot", func(t *testing.T) {
				testFFmpegScreenshot(t, ctx, client)
			})
			t.Run("CDPMonitorScreenshot", func(t *testing.T) {
				testFFmpegCDPMonitorScreenshot(t, ctx, c)
			})
			// The next two need megabytes of encoded output: one to hit the size
			// limit, the other because ffmpeg only writes fragments to disk once
			// its ~32 KB output buffer fills, which a near-static screen can take
			// minutes to do.
			t.Run("RecordingInProgressAndForceStop", func(t *testing.T) {
				if !img.rendersToDisplay {
					t.Skip("needs on-screen motion; headless Chromium does not paint to the X display")
				}
				showScreenNoise(t, ctx, client)
				testFFmpegRecordingInProgressAndForceStop(t, ctx, c, client)
			})
			t.Run("RecordingMaxFileSize", func(t *testing.T) {
				if !img.rendersToDisplay {
					t.Skip("needs on-screen motion; headless Chromium does not paint to the X display")
				}
				showScreenNoise(t, ctx, client)
				testFFmpegRecordingMaxFileSize(t, ctx, c, client)
			})
			// Resizes the display, so it must run last.
			t.Run("RecordingOddDimensions", func(t *testing.T) {
				testFFmpegRecordingOddDimensions(t, ctx, c, client)
			})
		})
	}
}

// testFFmpegCapabilities asserts the binary still provides every encoder,
// (de)muxer and filter the server's ffmpeg invocations depend on. It fails
// fast with the missing component's name, ahead of the behavioural tests.
func testFFmpegCapabilities(t *testing.T, ctx context.Context, c *TestContainer) {
	// cmd/api/main.go refuses to start without a working `ffmpeg -version`.
	for _, bin := range []string{"ffmpeg", "ffprobe"} {
		code, out, err := c.Exec(ctx, []string{bin, "-hide_banner", "-version"})
		require.NoError(t, err)
		require.Zero(t, code, "%s -version failed: %s", bin, out)
		t.Logf("%s", strings.SplitN(out, "\n", 2)[0])
	}

	required := map[string][]string{
		// libx264: recording video; aac: recording audio; png: screenshot API
		"encoders": {"libx264", "aac", "png"},
		// x11grab: all capture; pulse: recording audio; ffmetadata: chapters at finalize
		"demuxers": {"x11grab", "pulse", "ffmetadata"},
		// mp4: recording and finalize remux; image2pipe: screenshot API; image2: cdpmonitor screenshots
		"muxers": {"mp4", "image2pipe", "image2"},
		// pad: odd-dimension recording; crop: screenshot regions; scale: cdpmonitor screenshots
		"filters": {"pad", "crop", "scale"},
	}
	for kind, names := range required {
		code, out, err := c.Exec(ctx, []string{"ffmpeg", "-hide_banner", "-" + kind})
		require.NoError(t, err)
		require.Zero(t, code, "ffmpeg -%s failed: %s", kind, out)
		for _, name := range names {
			// Rows look like " V....D libx264  ...", " D d pulse  ..." or " ..C crop  ...".
			row := regexp.MustCompile(`(?m)^\s*[A-Za-z.]+(?:\s+d)?\s+` + regexp.QuoteMeta(name) + `\s`)
			require.True(t, row.MatchString(out), "ffmpeg build is missing %s %q (see `ffmpeg -%s`)", strings.TrimSuffix(kind, "s"), name, kind)
		}
	}
}

// testFFmpegRecording covers the default production recording path: capture
// flags, encoder settings, graceful SIGINT stop and the faststart remux.
func testFFmpegRecording(t *testing.T, ctx context.Context, c *TestContainer, client *instanceoapi.ClientWithResponses) {
	id := "ffmpeg-default"
	started := startFFmpegRecording(t, ctx, client, instanceoapi.StartRecordingJSONRequestBody{Id: &id})
	time.Sleep(4 * time.Second)
	stopStarted := time.Now()
	stopFFmpegRecording(t, ctx, client, id, false)
	elapsed := stopStarted.Sub(started)
	require.Less(t, time.Since(stopStarted), ffmpegMaxGracefulStop, "graceful stop was slow; ffmpeg may be ignoring SIGINT")

	data := downloadFFmpegRecording(t, ctx, client, id)
	boxes := mp4TopLevelBoxes(t, data, false)
	require.NotContains(t, boxes, "moof", "finalized recording should not be fragmented: %v", boxes)
	requireBoxBefore(t, boxes, "moov", "mdat", "finalized recording should be faststart")

	path := writeFFmpegContainerFile(t, ctx, c, id, data)
	probe := ffprobeFile(t, ctx, c, path)
	video := probe.stream(t, "video")
	require.Equal(t, "h264", video.CodecName)
	require.Equal(t, "High", video.Profile)
	require.Equal(t, "yuv420p", video.PixFmt)
	require.Equal(t, ffmpegTestWidth, video.Width)
	require.Equal(t, ffmpegTestHeight, video.Height)
	requireFrameRateNear(t, video.AvgFrameRate, ffmpegDefaultFrameRate)
	require.Nil(t, probe.streamOrNil("audio"), "audio is opt-in and was not requested")

	duration := probe.duration(t)
	require.InDelta(t, elapsed.Seconds(), duration, 2, "recording duration should match wall-clock recording time")
	requireVideoTimeline(t, ctx, c, path, duration)
}

func testFFmpegRecordingExplicitFrameRate(t *testing.T, ctx context.Context, c *TestContainer, client *instanceoapi.ClientWithResponses) {
	id := "ffmpeg-framerate"
	frameRate := 25
	startFFmpegRecording(t, ctx, client, instanceoapi.StartRecordingJSONRequestBody{Id: &id, Framerate: &frameRate})
	time.Sleep(3 * time.Second)
	stopFFmpegRecording(t, ctx, client, id, false)

	path := writeFFmpegContainerFile(t, ctx, c, id, downloadFFmpegRecording(t, ctx, client, id))
	requireFrameRateNear(t, ffprobeFile(t, ctx, c, path).stream(t, "video").AvgFrameRate, frameRate)
}

// testFFmpegRecordingAudio checks the pulse input, -isync and AAC settings
// produce the expected track. Audible-content and A/V-sync checks live in
// TestReplayRecordingIncludesAudioTrack.
func testFFmpegRecordingAudio(t *testing.T, ctx context.Context, c *TestContainer, client *instanceoapi.ClientWithResponses) {
	id := "ffmpeg-audio"
	recordAudio := true
	startFFmpegRecording(t, ctx, client, instanceoapi.StartRecordingJSONRequestBody{Id: &id, RecordAudio: &recordAudio})
	time.Sleep(3 * time.Second)
	stopStarted := time.Now()
	stopFFmpegRecording(t, ctx, client, id, false)
	require.Less(t, time.Since(stopStarted), ffmpegMaxGracefulStop, "graceful stop was slow; ffmpeg may be ignoring SIGINT")

	path := writeFFmpegContainerFile(t, ctx, c, id, downloadFFmpegRecording(t, ctx, client, id))
	probe := ffprobeFile(t, ctx, c, path)
	require.Equal(t, "h264", probe.stream(t, "video").CodecName)
	audio := probe.stream(t, "audio")
	require.Equal(t, "aac", audio.CodecName)
	require.Equal(t, "48000", audio.SampleRate)
	require.Equal(t, 2, audio.Channels)
}

// testFFmpegRecordingInProgressAndForceStop checks the data-safety contract:
// the file is a readable fragmented MP4 while ffmpeg is still writing it, and
// a SIGKILLed recording still finalizes into something playable.
func testFFmpegRecordingInProgressAndForceStop(t *testing.T, ctx context.Context, c *TestContainer, client *instanceoapi.ClientWithResponses) {
	id := "ffmpeg-inprogress"
	startFFmpegRecording(t, ctx, client, instanceoapi.StartRecordingJSONRequestBody{Id: &id})

	// Past the 2s -frag_duration so at least one fragment has been flushed.
	time.Sleep(5 * time.Second)
	var partial []byte
	require.Eventually(t, func() bool {
		rsp, err := client.DownloadRecordingWithResponse(ctx, &instanceoapi.DownloadRecordingParams{Id: &id})
		if err != nil || rsp.StatusCode() != http.StatusOK {
			return false
		}
		partial = rsp.Body
		return true
	}, 15*time.Second, 500*time.Millisecond, "in-progress recording never became downloadable")

	// A mid-write download can legitimately end part-way through a box.
	boxes := mp4TopLevelBoxes(t, partial, true)
	requireBoxBefore(t, boxes, "moov", "moof", "in-progress recording should be a fragmented MP4 with an empty moov up front")
	partialPath := writeFFmpegContainerFile(t, ctx, c, id+"-partial", partial)
	require.Equal(t, "h264", ffprobeFile(t, ctx, c, partialPath).stream(t, "video").CodecName)

	stopFFmpegRecording(t, ctx, client, id, true)
	path := writeFFmpegContainerFile(t, ctx, c, id, downloadFFmpegRecording(t, ctx, client, id))
	probe := ffprobeFile(t, ctx, c, path)
	require.Equal(t, "h264", probe.stream(t, "video").CodecName)
	require.Greater(t, probe.duration(t), 1.0, "force-stopped recording should keep its flushed fragments")
}

func testFFmpegRecordingMaxDuration(t *testing.T, ctx context.Context, c *TestContainer, client *instanceoapi.ClientWithResponses) {
	id := "ffmpeg-maxduration"
	maxDuration := 3
	startFFmpegRecording(t, ctx, client, instanceoapi.StartRecordingJSONRequestBody{Id: &id, MaxDurationInSeconds: &maxDuration})
	waitForRecordingToEnd(t, ctx, client, id, 30*time.Second)

	path := writeFFmpegContainerFile(t, ctx, c, id, downloadFFmpegRecording(t, ctx, client, id))
	require.InDelta(t, float64(maxDuration), ffprobeFile(t, ctx, c, path).duration(t), 1.5, "-t should cap the recording duration")
}

func testFFmpegRecordingMaxFileSize(t *testing.T, ctx context.Context, c *TestContainer, client *instanceoapi.ClientWithResponses) {
	id := "ffmpeg-maxsize"
	maxSizeMB := 1
	startFFmpegRecording(t, ctx, client, instanceoapi.StartRecordingJSONRequestBody{Id: &id, MaxFileSizeInMB: &maxSizeMB})
	waitForRecordingToEnd(t, ctx, client, id, 60*time.Second)

	data := downloadFFmpegRecording(t, ctx, client, id)
	require.Greater(t, len(data), maxSizeMB*1024*1024, "recording should have reached the size limit")
	path := writeFFmpegContainerFile(t, ctx, c, id, data)
	probe := ffprobeFile(t, ctx, c, path)
	require.Equal(t, "h264", probe.stream(t, "video").CodecName)
	// -fs only sees bytes once a fragment is flushed, so the file overshoots the
	// limit by up to one -frag_duration (2s) of video. What matters is that ffmpeg
	// stopped soon after crossing it rather than running on.
	require.Less(t, probe.duration(t), 6.0, "-fs should end the recording shortly after the limit is crossed")
}

// testFFmpegRecordingChapters covers the ffmetadata input finalize adds for
// markers. A failing chaptered remux silently falls back to no chapters, so
// only asserting on the chapters themselves catches a break here.
func testFFmpegRecordingChapters(t *testing.T, ctx context.Context, c *TestContainer, client *instanceoapi.ClientWithResponses) {
	id := "ffmpeg-chapters"
	startFFmpegRecording(t, ctx, client, instanceoapi.StartRecordingJSONRequestBody{Id: &id})
	names := []string{"alpha", "beta"}
	for _, name := range names {
		time.Sleep(time.Second)
		rsp, err := client.MarkRecordingWithResponse(ctx, instanceoapi.MarkRecordingJSONRequestBody{Id: &id, Name: name})
		require.NoError(t, err)
		require.Equal(t, http.StatusCreated, rsp.StatusCode(), "mark %q: %s body=%s", name, rsp.Status(), string(rsp.Body))
	}
	time.Sleep(time.Second)
	stopFFmpegRecording(t, ctx, client, id, false)

	path := writeFFmpegContainerFile(t, ctx, c, id, downloadFFmpegRecording(t, ctx, client, id))
	// Finalize prepends a "_recording_start" chapter because ffmpeg forces the
	// first chapter to start at 0 (see recorder.sentinelChapterName).
	want := append([]string{"_recording_start"}, names...)
	chapters := ffprobeFile(t, ctx, c, path).Chapters
	require.Len(t, chapters, len(want), "finalized recording should carry the start chapter plus one per marker")
	prevStart := -1.0
	for i, ch := range chapters {
		require.Equal(t, want[i], ch.Tags["title"])
		start, err := strconv.ParseFloat(ch.StartTime, 64)
		require.NoError(t, err)
		require.Greater(t, start, prevStart, "chapters should be in marker order")
		prevStart = start
	}
}

func testFFmpegScreenshot(t *testing.T, ctx context.Context, client *instanceoapi.ClientWithResponses) {
	full, err := client.TakeScreenshotWithResponse(ctx, instanceoapi.TakeScreenshotJSONRequestBody{})
	require.NoError(t, err)
	require.Equal(t, http.StatusOK, full.StatusCode(), "screenshot: %s body=%s", full.Status(), string(full.Body))
	requirePNGSize(t, full.Body, ffmpegTestWidth, ffmpegTestHeight)

	region := instanceoapi.ScreenshotRegion{X: 10, Y: 20, Width: 200, Height: 100}
	cropped, err := client.TakeScreenshotWithResponse(ctx, instanceoapi.TakeScreenshotJSONRequestBody{Region: &region})
	require.NoError(t, err)
	require.Equal(t, http.StatusOK, cropped.StatusCode(), "region screenshot: %s body=%s", cropped.Status(), string(cropped.Body))
	requirePNGSize(t, cropped.Body, region.Width, region.Height)
}

// testFFmpegCDPMonitorScreenshot runs cdpmonitor's exact ffmpeg arguments at
// full size and with the downscale filter it falls back to for large frames.
func testFFmpegCDPMonitorScreenshot(t *testing.T, ctx context.Context, c *TestContainer) {
	for _, divisor := range []int{1, 2} {
		args := cdpmonitor.FFmpegScreenshotArgs(1, divisor)
		// Exec merges stdout and stderr, so base64 the image and drop ffmpeg's log.
		code, out, err := c.Exec(ctx, append([]string{"bash", "-c", `set -o pipefail; ffmpeg -loglevel error "$@" 2>/dev/null | base64 -w0`, "ffmpeg"}, args...))
		require.NoError(t, err)
		require.Zero(t, code, "cdpmonitor screenshot (divisor %d) failed: %s", divisor, out)
		img, err := base64.StdEncoding.DecodeString(strings.TrimSpace(out))
		require.NoError(t, err, "decode base64 screenshot")

		cfg, format, err := image.DecodeConfig(bytes.NewReader(img))
		require.NoError(t, err, "cdpmonitor screenshot (divisor %d) is not a decodable image", divisor)
		// The image2 muxer picks the codec, which is mjpeg today even though the
		// event field is named Png and documented as PNG. Pinned so an ffmpeg
		// default change is caught; update alongside any fix to that mismatch.
		require.Equal(t, "jpeg", format, "cdpmonitor screenshot format changed (divisor %d)", divisor)
		require.Equal(t, ffmpegTestWidth/divisor, cfg.Width)
		require.Equal(t, ffmpegTestHeight/divisor, cfg.Height)
	}
}

// testFFmpegRecordingOddDimensions covers the pad filter that keeps libx264
// working (yuv420p needs even dimensions) when the display has an odd size.
func testFFmpegRecordingOddDimensions(t *testing.T, ctx context.Context, c *TestContainer, client *instanceoapi.ClientWithResponses) {
	reqW, reqH := 1279, 719
	rate := instanceoapi.PatchDisplayRequestRefreshRate(60)
	rsp, err := client.PatchDisplayWithResponse(ctx, instanceoapi.PatchDisplayJSONRequestBody{Width: &reqW, Height: &reqH, RefreshRate: &rate})
	require.NoError(t, err)
	if rsp.StatusCode() != http.StatusOK {
		// Headful Xorg without neko only offers its predefined modes.
		t.Skipf("display cannot be set to an odd size here: %s body=%s", rsp.Status(), string(rsp.Body))
	}

	// The display may round the request (headful Xorg uses libxcvt's 8-pixel grid).
	require.Eventually(t, func() bool {
		w, h, err := getXRootResolution(ctx, c)
		return err == nil && (w != ffmpegTestWidth || h != ffmpegTestHeight)
	}, 30*time.Second, 250*time.Millisecond, "display never left %dx%d", ffmpegTestWidth, ffmpegTestHeight)
	rootW, rootH, err := getXRootResolution(ctx, c)
	require.NoError(t, err)
	if rootW%2 == 0 && rootH%2 == 0 {
		t.Skipf("display realized %dx%d for a %dx%d request; no odd dimension to pad", rootW, rootH, reqW, reqH)
	}

	id := "ffmpeg-odd"
	startFFmpegRecording(t, ctx, client, instanceoapi.StartRecordingJSONRequestBody{Id: &id})
	time.Sleep(3 * time.Second)
	stopFFmpegRecording(t, ctx, client, id, false)

	path := writeFFmpegContainerFile(t, ctx, c, id, downloadFFmpegRecording(t, ctx, client, id))
	video := ffprobeFile(t, ctx, c, path).stream(t, "video")
	require.Equal(t, rootW+rootW%2, video.Width, "odd width should be padded to even")
	require.Equal(t, rootH+rootH%2, video.Height, "odd height should be padded to even")
}

// showScreenNoise fills the browser viewport with per-frame random pixels.
// Noise is close to incompressible, so libx264 output grows by megabytes within
// seconds. The page is reset when the calling test finishes.
func showScreenNoise(t *testing.T, ctx context.Context, client *instanceoapi.ClientWithResponses) {
	t.Helper()
	t.Cleanup(func() {
		ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
		defer cancel()
		runPlaywright(t, ctx, client, `await page.goto('about:blank');`)
	})
	runPlaywright(t, ctx, client, `
		await page.setContent('<canvas id="c" style="position:fixed;inset:0;width:100vw;height:100vh"></canvas>');
		await page.evaluate(() => {
			const canvas = document.getElementById('c');
			canvas.width = innerWidth;
			canvas.height = innerHeight;
			const ctx = canvas.getContext('2d');
			const frame = ctx.createImageData(canvas.width, canvas.height);
			const draw = () => {
				const px = new Uint32Array(frame.data.buffer);
				for (let i = 0; i < px.length; i++) px[i] = (Math.random() * 0xffffffff) | 0xff000000;
				ctx.putImageData(frame, 0, 0);
				requestAnimationFrame(draw);
			};
			draw();
		});
	`)
}

func runPlaywright(t *testing.T, ctx context.Context, client *instanceoapi.ClientWithResponses, code string) {
	t.Helper()
	rsp, err := client.ExecutePlaywrightCodeWithResponse(ctx, instanceoapi.ExecutePlaywrightCodeJSONRequestBody{Code: code})
	require.NoError(t, err, "playwright request failed")
	require.Equal(t, http.StatusOK, rsp.StatusCode(), "unexpected playwright status: %s body=%s", rsp.Status(), string(rsp.Body))
	require.NotNil(t, rsp.JSON200)
	require.True(t, rsp.JSON200.Success, "playwright failed: %s stderr=%s", stringValue(rsp.JSON200.Error), stringValue(rsp.JSON200.Stderr))
}

func startFFmpegRecording(t *testing.T, ctx context.Context, client *instanceoapi.ClientWithResponses, body instanceoapi.StartRecordingJSONRequestBody) time.Time {
	t.Helper()
	started := time.Now()
	rsp, err := client.StartRecordingWithResponse(ctx, body)
	require.NoError(t, err, "POST /recording/start failed")
	require.Equal(t, http.StatusCreated, rsp.StatusCode(), "start recording: %s body=%s", rsp.Status(), string(rsp.Body))
	// If the test fails mid-recording, don't leave ffmpeg running into the next
	// subtest. Stopping an already-stopped recording is a harmless no-op here.
	t.Cleanup(func() {
		ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
		defer cancel()
		force := true
		_, _ = client.StopRecordingWithResponse(ctx, instanceoapi.StopRecordingJSONRequestBody{Id: body.Id, ForceStop: &force})
	})
	return started
}

func stopFFmpegRecording(t *testing.T, ctx context.Context, client *instanceoapi.ClientWithResponses, id string, force bool) {
	t.Helper()
	rsp, err := client.StopRecordingWithResponse(ctx, instanceoapi.StopRecordingJSONRequestBody{Id: &id, ForceStop: &force})
	require.NoError(t, err, "POST /recording/stop failed")
	require.Equal(t, http.StatusOK, rsp.StatusCode(), "stop recording: %s body=%s", rsp.Status(), string(rsp.Body))
}

func downloadFFmpegRecording(t *testing.T, ctx context.Context, client *instanceoapi.ClientWithResponses, id string) []byte {
	t.Helper()
	rsp, err := client.DownloadRecordingWithResponse(ctx, &instanceoapi.DownloadRecordingParams{Id: &id})
	require.NoError(t, err, "GET /recording/download failed")
	require.Equal(t, http.StatusOK, rsp.StatusCode(), "download recording: %s body=%s", rsp.Status(), string(rsp.Body))
	require.NotEmpty(t, rsp.Body, "downloaded recording is empty")
	return rsp.Body
}

// waitForRecordingToEnd waits for ffmpeg to exit on its own (duration or size limit).
func waitForRecordingToEnd(t *testing.T, ctx context.Context, client *instanceoapi.ClientWithResponses, id string, timeout time.Duration) {
	t.Helper()
	require.Eventually(t, func() bool {
		rsp, err := client.ListRecordersWithResponse(ctx)
		if err != nil || rsp.JSON200 == nil {
			return false
		}
		for _, r := range *rsp.JSON200 {
			if r.Id == id {
				return !r.IsRecording
			}
		}
		return false
	}, timeout, 250*time.Millisecond, "recording %q never stopped on its own", id)
}

// writeFFmpegContainerFile writes downloaded bytes back into the container so
// the image's own ffprobe inspects exactly what API clients receive.
func writeFFmpegContainerFile(t *testing.T, ctx context.Context, c *TestContainer, name string, data []byte) string {
	t.Helper()
	client, err := c.APIClient()
	require.NoError(t, err)
	path := "/tmp/ffmpeg-e2e-" + name + ".mp4"
	rsp, err := client.WriteFileWithBodyWithResponse(ctx, &instanceoapi.WriteFileParams{Path: path}, "video/mp4", bytes.NewReader(data))
	require.NoError(t, err, "write %s", path)
	require.Equal(t, http.StatusCreated, rsp.StatusCode(), "write %s: %s body=%s", path, rsp.Status(), string(rsp.Body))
	return path
}

type ffprobeStream struct {
	CodecType    string `json:"codec_type"`
	CodecName    string `json:"codec_name"`
	Profile      string `json:"profile"`
	PixFmt       string `json:"pix_fmt"`
	Width        int    `json:"width"`
	Height       int    `json:"height"`
	AvgFrameRate string `json:"avg_frame_rate"`
	SampleRate   string `json:"sample_rate"`
	Channels     int    `json:"channels"`
}

type ffprobeResult struct {
	Streams []ffprobeStream `json:"streams"`
	Format  struct {
		Duration string `json:"duration"`
	} `json:"format"`
	Chapters []struct {
		StartTime string            `json:"start_time"`
		Tags      map[string]string `json:"tags"`
	} `json:"chapters"`
}

func (p ffprobeResult) streamOrNil(codecType string) *ffprobeStream {
	for i := range p.Streams {
		if p.Streams[i].CodecType == codecType {
			return &p.Streams[i]
		}
	}
	return nil
}

func (p ffprobeResult) stream(t *testing.T, codecType string) ffprobeStream {
	t.Helper()
	s := p.streamOrNil(codecType)
	require.NotNil(t, s, "no %s stream in %+v", codecType, p.Streams)
	return *s
}

func (p ffprobeResult) duration(t *testing.T) float64 {
	t.Helper()
	d, err := strconv.ParseFloat(p.Format.Duration, 64)
	require.NoError(t, err, "parse format duration %q", p.Format.Duration)
	return d
}

func ffprobeFile(t *testing.T, ctx context.Context, c *TestContainer, path string) ffprobeResult {
	t.Helper()
	code, out, err := c.Exec(ctx, []string{"ffprobe", "-v", "error", "-show_format", "-show_streams", "-show_chapters", "-of", "json", path})
	require.NoError(t, err)
	require.Zero(t, code, "ffprobe %s failed: %s", path, out)
	var probe ffprobeResult
	require.NoError(t, json.Unmarshal([]byte(out), &probe), "parse ffprobe output: %s", out)
	return probe
}

// requireVideoTimeline checks the timestamp flags still yield a zero-based
// timeline whose decode order only moves forward and ends within the container
// duration. dts is used because B-frames legitimately reorder pts.
func requireVideoTimeline(t *testing.T, ctx context.Context, c *TestContainer, path string, duration float64) {
	t.Helper()
	code, out, err := c.Exec(ctx, []string{"ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time,dts_time", "-of", "csv=p=0", path})
	require.NoError(t, err)
	require.Zero(t, code, "ffprobe packets %s failed: %s", path, out)

	var prevDTS, minPTS, maxPTS float64
	lines := strings.Fields(out)
	require.NotEmpty(t, lines, "no video packets")
	for i, line := range lines {
		ptsStr, dtsStr, ok := strings.Cut(strings.TrimSuffix(line, ","), ",")
		require.True(t, ok, "unexpected packet row %q", line)
		pts, err := strconv.ParseFloat(ptsStr, 64)
		require.NoError(t, err, "parse pts in %q", line)
		dts, err := strconv.ParseFloat(dtsStr, 64)
		require.NoError(t, err, "parse dts in %q", line)
		if i == 0 {
			minPTS, maxPTS = pts, pts
		} else {
			require.Greater(t, dts, prevDTS, "decode timestamps must increase (packet %d: %q)", i, line)
		}
		prevDTS = dts
		minPTS, maxPTS = min(minPTS, pts), max(maxPTS, pts)
	}
	require.GreaterOrEqual(t, minPTS, 0.0, "video timestamps should not be negative")
	require.Less(t, minPTS, 1.0, "video timeline should start near zero")
	require.LessOrEqual(t, maxPTS, duration+0.5, "video timestamps run past the container duration")
}

func requireFrameRateNear(t *testing.T, rate string, want int) {
	t.Helper()
	num, den, ok := strings.Cut(rate, "/")
	require.True(t, ok, "unexpected frame rate %q", rate)
	n, err := strconv.ParseFloat(num, 64)
	require.NoError(t, err)
	d, err := strconv.ParseFloat(den, 64)
	require.NoError(t, err)
	require.NotZero(t, d, "frame rate %q has zero denominator", rate)
	require.InEpsilon(t, float64(want), n/d, 0.25, "average frame rate %s should be within 25%% of %d", rate, want)
}

func requirePNGSize(t *testing.T, data []byte, width, height int) {
	t.Helper()
	cfg, err := png.DecodeConfig(bytes.NewReader(data))
	require.NoError(t, err, "screenshot is not a PNG")
	require.Equal(t, width, cfg.Width)
	require.Equal(t, height, cfg.Height)
}

// mp4TopLevelBoxes returns the types of the file's top-level ISO BMFF boxes in
// order. allowTruncated tolerates a final box that runs past the end of data.
func mp4TopLevelBoxes(t *testing.T, data []byte, allowTruncated bool) []string {
	t.Helper()
	var boxes []string
	total := uint64(len(data))
	for off := uint64(0); off+8 <= total; {
		size := uint64(binary.BigEndian.Uint32(data[off:]))
		boxType := string(data[off+4 : off+8])
		header := uint64(8)
		switch size {
		case 0: // box extends to end of file
			size = total - off
		case 1: // 64-bit size follows the type
			require.LessOrEqual(t, off+16, total, "truncated %s box header at %d", boxType, off)
			size = binary.BigEndian.Uint64(data[off+8:])
			header = 16
		}
		require.GreaterOrEqual(t, size, header, "invalid %s box size %d at %d", boxType, size, off)
		boxes = append(boxes, boxType)
		if size > total-off {
			require.True(t, allowTruncated, "%s box at %d (size %d) runs past the end of the %d-byte file", boxType, off, size, total)
			break
		}
		off += size
	}
	require.NotEmpty(t, boxes, "no MP4 boxes found")
	return boxes
}

func requireBoxBefore(t *testing.T, boxes []string, first, second, msg string) {
	t.Helper()
	i, j := slices.Index(boxes, first), slices.Index(boxes, second)
	require.True(t, i >= 0 && j >= 0 && i < j, "%s: want %s before %s, got %v", msg, first, second, boxes)
}

func dumpAPILogOnFailure(t *testing.T, c *TestContainer) {
	if !t.Failed() {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	code, out, err := c.Exec(ctx, []string{"tail", "-n", "200", "/var/log/supervisord/kernel-images-api"})
	if err != nil || code != 0 {
		t.Logf("could not read kernel-images-api log: code=%d err=%v", code, err)
		return
	}
	t.Logf("kernel-images-api log (last 200 lines):\n%s", out)
}
