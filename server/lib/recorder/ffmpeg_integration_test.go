// FFmpeg integration tests to help identify incompatibilities across version changes

//go:build integration

package recorder

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"image/png"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/onkernel/kernel-images/server/lib/scaletozero"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type ffprobeStreamInfo struct {
	CodecName    string `json:"codec_name"`
	CodecType    string `json:"codec_type"`
	Profile      string `json:"profile"`
	PixFmt       string `json:"pix_fmt"`
	Width        int    `json:"width"`
	Height       int    `json:"height"`
	RFrameRate   string `json:"r_frame_rate"`
	AvgFrameRate string `json:"avg_frame_rate"`
	DurationTS   int64  `json:"duration_ts"`
	Duration     string `json:"duration"`
	NbFrames     string `json:"nb_frames"`
}

type ffprobeFormatInfo struct {
	Filename       string            `json:"filename"`
	NbStreams      int               `json:"nb_streams"`
	FormatName     string            `json:"format_name"`
	FormatLongName string            `json:"format_long_name"`
	Duration       string            `json:"duration"`
	Size           string            `json:"size"`
	Tags           map[string]string `json:"tags"`
}

type ffprobeOutput struct {
	Streams []ffprobeStreamInfo `json:"streams"`
	Format  ffprobeFormatInfo   `json:"format"`
}

type videoValidationOpts struct {
	minDuration               time.Duration
	maxDuration               time.Duration
	expectedWidth             int
	expectedHeight            int
	expectedFrameRate         int
	frameRatePercentTolerance float64 // tolerance for frame rate (e.g., 10 = +-10%)
}

// Ensures the DISPLAY environment variable is set and non-empty or fails the test
func requireDisplayEnvironmentVariable(t *testing.T) string {
	t.Helper()

	display := os.Getenv("DISPLAY")
	if display == "" {
		t.Fatal("DISPLAY environment variable required")
	}

	return display
}

// Extracts the display number (e.g. 1 from :1) from the DISPLAY environment variable
func extractDisplayNumberFromEnvironmentVariable(t *testing.T) int {
	t.Helper()

	display := requireDisplayEnvironmentVariable(t)

	// Parse :N or :N.S format
	parts := strings.Split(display, ":")
	if len(parts) < 2 {
		t.Fatal("Invalid DISPLAY environment variable format after splitting on ':'")
	}

	numStr := strings.Split(parts[1], ".")[0]
	num, err := strconv.Atoi(numStr)
	if err != nil {
		t.Fatalf("Invalid DISPLAY environment variable format after splitting on '.': %s", err)
	}

	return num
}

func setupTestRecorder(t *testing.T, id string, opts *FFmpegRecordingParams) *FFmpegRecorder {
	t.Helper()

	outputDir := t.TempDir()
	displayNum := extractDisplayNumberFromEnvironmentVariable(t)
	frameRate := 10
	maxSizeMB := 50
	logLevel := "quiet"

	params := FFmpegRecordingParams{
		OutputDir:   &outputDir,
		DisplayNum:  &displayNum,
		FrameRate:   &frameRate,
		MaxSizeInMB: &maxSizeMB,
		LogLevel:    &logLevel,
	}

	// Apply overrides if provided
	if opts != nil {
		if opts.FrameRate != nil {
			params.FrameRate = opts.FrameRate
		}
		if opts.DisplayNum != nil {
			params.DisplayNum = opts.DisplayNum
		}
		if opts.MaxSizeInMB != nil {
			params.MaxSizeInMB = opts.MaxSizeInMB
		}
		if opts.MaxDurationInSeconds != nil {
			params.MaxDurationInSeconds = opts.MaxDurationInSeconds
		}
		if opts.OutputDir != nil {
			params.OutputDir = opts.OutputDir
		}
		if opts.LogLevel != nil {
			params.LogLevel = opts.LogLevel
		}
	}

	ffmpegPath := getFFmpegBinaryPath(t)
	outputPath := filepath.Join(*params.OutputDir, fmt.Sprintf("%s.mp4", id))

	return &FFmpegRecorder{
		id:         id,
		binaryPath: ffmpegPath,
		outputPath: outputPath,
		params:     params,
		stz:        scaletozero.NewOncer(scaletozero.NewNoopController()),
		exitCode:   exitCodeInitValue,
		exited:     make(chan struct{}),
	}
}

func getFFmpegBinaryPath(t *testing.T) string {
	t.Helper()

	if path := os.Getenv("FFMPEG_PATH"); path != "" {
		return path
	}

	return "ffmpeg"
}

func getFFprobeBinaryPath(t *testing.T) string {
	t.Helper()

	if path := os.Getenv("FFPROBE_PATH"); path != "" {
		return path
	}

	return "ffprobe"
}

func getXrandrBinaryPath(t *testing.T) string {
	t.Helper()

	if path := os.Getenv("XRANDR_PATH"); path != "" {
		return path
	}

	return "xrandr"
}

// Checks basic recording output format and optionally against the provided specification
func validateMP4Recording(t *testing.T, recordingPath string, specification videoValidationOpts) {
	t.Helper()

	require.FileExists(t, recordingPath, "recording file should exist")

	cmd := exec.Command(getFFprobeBinaryPath(t),
		"-v", "quiet",
		"-print_format", "json",
		"-show_format",
		"-show_streams",
		recordingPath,
	)

	output, err := cmd.Output()
	require.NoError(t, err, "ffprobe should succeed")

	var probe ffprobeOutput
	err = json.Unmarshal(output, &probe)
	require.NoError(t, err, "ffprobe output should be valid JSON")

	var videoStream *ffprobeStreamInfo
	for i := range probe.Streams {
		if probe.Streams[i].CodecType == "video" {
			videoStream = &probe.Streams[i]
			break
		}
	}
	require.NotNil(t, videoStream, "should have video stream")

	// Basic case video format validation
	assert.Equal(t, "h264", videoStream.CodecName, "should use H.264 codec")
	assert.Equal(t, "High", videoStream.Profile, "should use High profile")
	assert.Equal(t, "yuv420p", videoStream.PixFmt, "should use yuv420p pixel format")
	assert.Contains(t, probe.Format.FormatName, "mp4", "should be MP4 format")

	// Optional specification validation
	if specification.expectedWidth > 0 {
		assert.Equal(t, specification.expectedWidth, videoStream.Width, "video width should match")
	}
	if specification.expectedHeight > 0 {
		assert.Equal(t, specification.expectedHeight, videoStream.Height, "video height should match")
	}

	if specification.expectedFrameRate > 0 {
		parts := strings.Split(videoStream.AvgFrameRate, "/")
		if len(parts) != 2 {
			t.Error("Unable to convert video stream frame rate ratio")
		}

		numerator, err1 := strconv.ParseFloat(parts[0], 64)
		denominator, err2 := strconv.ParseFloat(parts[1], 64)

		if err1 != nil || err2 != nil || denominator == 0 {
			t.Error("Unable to extract video stream frame rate ratio numerator or denominator")
		}
		measuredAverageFrameRate := numerator / denominator

		expectedMin := float64(specification.expectedFrameRate) * (1 - specification.frameRatePercentTolerance/100)
		expectedMax := float64(specification.expectedFrameRate) * (1 + specification.frameRatePercentTolerance/100)
		assert.GreaterOrEqual(t, measuredAverageFrameRate, expectedMin, "frame rate should be at least tolerance minimum")
		assert.LessOrEqual(t, measuredAverageFrameRate, expectedMax, "frame rate should be at most tolerance maximum")
	}

	if specification.minDuration > 0 || specification.maxDuration > 0 {
		seconds, err := strconv.ParseFloat(probe.Format.Duration, 64)
		if err != nil {
			t.Error("Unable to parse duration string")
		}
		duration := time.Duration(seconds * float64(time.Second))

		if specification.minDuration > 0 {
			assert.GreaterOrEqual(t, duration, specification.minDuration, "duration should be at least minimum")
		}
		if specification.maxDuration > 0 {
			assert.LessOrEqual(t, duration, specification.maxDuration, "duration should be at most maximum")
		}
	}

	t.Logf("Recording validation passed: codec=%s, profile=%s, pix_fmt=%s, dimensions=%dx%d, duration=%s",
		videoStream.CodecName, videoStream.Profile, videoStream.PixFmt,
		videoStream.Width, videoStream.Height, probe.Format.Duration)
}

func hasFragmentedMP4Structure(t *testing.T, path string) bool {
	t.Helper()

	// Fragmented MP4 files will have moof atom byte sequences
	data, err := os.ReadFile(path)
	require.NoError(t, err)
	return bytes.Contains(data, []byte("moof"))
}

// Checks that video timestamps are present, monotonically increasing, and positive
func validateTimestamps(t *testing.T, path string) {
	t.Helper()

	cmd := exec.Command(getFFprobeBinaryPath(t),
		"-v", "quiet",
		"-select_streams", "v:0",
		"-show_entries", "frame=pts_time",
		"-of", "default=noprint_wrappers=1:nokey=1",
		path,
	)

	output, err := cmd.Output()
	require.NoError(t, err, "ffprobe should succeed in listing frame times")

	var previousPTS float64 = -1.0
	{
		var i int64
		for line := range strings.SplitSeq(string(output), "\n") {
			line = strings.TrimSpace(line)
			if line == "" {
				t.Log("Timestamp line is empty, skipping to next line")
				continue
			}

			pts, err := strconv.ParseFloat(line, 64)
			if err != nil {
				t.Log("Unable to convert timestamp to float, skipping to next timestamp")
				continue
			}

			assert.GreaterOrEqual(t, pts, 0.0, "timestamp should not be negative")

			if i > 0 {
				assert.GreaterOrEqual(t, pts, previousPTS, "timestamps should be monotonically increasing")
			}

			previousPTS = pts
			i = i + 1
		}
	}

	require.GreaterOrEqual(t, previousPTS, 0.0, "should have at least one frame of timing data")
}

func validatePNGScreenshot(t *testing.T, data []byte, expectedWidth, expectedHeight int) {
	t.Helper()

	require.NotEmpty(t, data, "screenshot data should not be empty")

	img, err := png.Decode(bytes.NewReader(data))
	require.NoError(t, err, "should be valid PNG")

	bounds := img.Bounds()
	width := bounds.Dx()
	height := bounds.Dy()

	if expectedWidth > 0 {
		assert.Equal(t, expectedWidth, width, "screenshot width should match")
	}
	if expectedHeight > 0 {
		assert.Equal(t, expectedHeight, height, "screenshot height should match")
	}

	t.Logf("Screenshot validated. PNG with dimensions: %dx%d", width, height)
}

// Extracts the resolution width and height of the display
func getScreenResolution(t *testing.T, display string) (width, height int) {
	t.Helper()

	cmd := exec.Command(getXrandrBinaryPath(t))
	cmd.Env = append(os.Environ(), fmt.Sprintf("DISPLAY=%s", display))
	output, err := cmd.Output()

	width, height = -1, -1

	if err == nil {
		for line := range strings.SplitSeq(string(output), "\n") {
			if strings.Contains(line, "*") && strings.Contains(line, "x") {
				parts := strings.Fields(line)
				dimensions, _, _ := strings.Cut(parts[0], "_")
				wString, hString, _ := strings.Cut(dimensions, "x")
				w, err1 := strconv.Atoi(wString)
				h, err2 := strconv.Atoi(hString)
				if err1 == nil && err2 == nil {
					width = w
					height = h
				} else {
					if err1 != nil {
						t.Errorf("Error extracting width from xrandr output: %s", err1)
					}
					if err2 != nil {
						t.Errorf("Error extracting height from xrandr output: %s", err2)
					}
				}
				break
			}
		}
	} else {
		t.Errorf("Error running command xrandr to extract dimensions: %s", err)
	}

	require.Greater(t, width, -1)
	require.Greater(t, height, -1)

	return width, height
}

// Stop the recorder, assert no errors, and ignore exit code 255 as it is expected
func stopAndAssertNoError(t *testing.T, recorder *FFmpegRecorder, ctx context.Context) error {
	t.Helper()
	err := recorder.Stop(ctx)
	if err != nil && recorder.exitCode == 255 && !recorder.IsRecording(ctx) {
		err = nil
		t.Log("ignoring exit code 255 as expected result of stopping via SIGINT")
	} else {
		assert.NoError(t, err)
	}
	return err
}

func TestXrandr_BinaryAvailability(t *testing.T) {
	t.Parallel()

	xrandrPath := getXrandrBinaryPath(t)
	t.Logf("Testing xrandr binary at: %s", xrandrPath)

	cmd := exec.Command(xrandrPath, "--help")
	err := cmd.Run()
	require.NoError(t, err, "xrandr should be available and executable")
}

func TestXrandr_ValidDisplay(t *testing.T) {
	t.Parallel()
	xrandrPath := getXrandrBinaryPath(t)
	t.Logf("Testing for valid display using xrandr and environment variable DISPLAY=%s", requireDisplayEnvironmentVariable(t))
	cmd := exec.Command(xrandrPath, "-q")
	err := cmd.Run()
	require.NoError(t, err, "failed to query display")
}

func TestFFmpeg_BinaryAvailability(t *testing.T) {
	t.Parallel()

	ffmpegPath := getFFmpegBinaryPath(t)
	t.Logf("Testing ffmpeg binary at: %s", ffmpegPath)

	cmd := exec.Command(ffmpegPath, "-version")
	err := cmd.Run()
	require.NoError(t, err, "ffmpeg should be available and executable")
}

func TestFFmpeg_VersionDetection(t *testing.T) {
	t.Parallel()

	ffmpegPath := getFFmpegBinaryPath(t)
	cmd := exec.Command(ffmpegPath, "-version")
	output, err := cmd.Output()
	require.NoError(t, err, "failed to get ffmpeg version")

	version := "unknown"
	lines := strings.Split(string(output), "\n")
	if len(lines) > 0 {
		version = strings.TrimSpace(lines[0])
	}

	t.Logf("FFmpeg version: %s", version)

	assert.NotEmpty(t, version, "should detect ffmpeg version")
	assert.Contains(t, strings.ToLower(version), "ffmpeg", "version string should contain 'ffmpeg'")
}

func TestFFmpeg_RequiredCodecs(t *testing.T) {
	t.Parallel()

	ffmpegPath := getFFmpegBinaryPath(t)

	cmd := exec.Command(ffmpegPath, "-codecs")
	output, err := cmd.Output()
	require.NoError(t, err, "should list codecs")

	codecList := string(output)
	assert.Contains(t, codecList, "libx264 ", "libx264 codec should be available")
	assert.Contains(t, codecList, "h264 ", "h264 codec should be available")
	assert.Contains(t, codecList, "png ", "PNG codec should be available")
	t.Log("All required codecs are available")
}

func TestFFmpeg_RequiredFormats(t *testing.T) {
	t.Parallel()

	ffmpegPath := getFFmpegBinaryPath(t)

	cmd := exec.Command(ffmpegPath, "-formats")
	output, err := cmd.Output()
	require.NoError(t, err, "should list formats")

	formatList := string(output)

	assert.Contains(t, formatList, "x11grab", "x11grab format should be available")
	assert.Contains(t, formatList, "image2pipe", "image2pipe format should be available")
	t.Log("All required formats are available")
}

func TestFFmpegRecording_VideoOutput(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	rec := setupTestRecorder(t, "video-output-test", nil)
	ctx := context.Background()
	err := rec.Start(ctx)

	require.NoError(t, err, "recording should start")

	time.Sleep(3 * time.Second)

	stopAndAssertNoError(t, rec, ctx)
	<-rec.exited

	validateMP4Recording(t, rec.outputPath, videoValidationOpts{
		minDuration:               2 * time.Second,
		maxDuration:               5 * time.Second,
		expectedFrameRate:         *rec.params.FrameRate,
		frameRatePercentTolerance: 20,
	})
}

func TestFFmpegRecording_CodecVerification(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	rec := setupTestRecorder(t, "codec-test", nil)

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)

	time.Sleep(2 * time.Second)
	stopAndAssertNoError(t, rec, ctx)
	<-rec.exited

	cmd := exec.Command(getFFprobeBinaryPath(t),
		"-v", "quiet",
		"-select_streams", "v:0",
		"-show_entries", "stream=codec_name,profile",
		"-of", "json",
		rec.outputPath,
	)

	output, err := cmd.Output()
	require.NoError(t, err)

	var probe ffprobeOutput
	err = json.Unmarshal(output, &probe)
	require.NoError(t, err)
	require.NotEmpty(t, probe.Streams)

	assert.Equal(t, "h264", probe.Streams[0].CodecName)
	assert.Equal(t, "High", probe.Streams[0].Profile)
}

func TestFFmpegRecording_PixelFormat(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	rec := setupTestRecorder(t, "pixfmt-test", nil)

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)

	time.Sleep(2 * time.Second)
	stopAndAssertNoError(t, rec, ctx)
	<-rec.exited

	cmd := exec.Command(getFFprobeBinaryPath(t),
		"-v", "quiet",
		"-select_streams", "v:0",
		"-show_entries", "stream=pix_fmt",
		"-of", "default=noprint_wrappers=1:nokey=1",
		rec.outputPath,
	)

	output, err := cmd.Output()
	require.NoError(t, err)

	pixelFormat := strings.TrimSpace(string(output))
	assert.Equal(t, "yuv420p", pixelFormat, "should use yuv420p pixel format")
}

func TestFFmpegRecording_FragmentedMP4(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	rec := setupTestRecorder(t, "fragmented-test", nil)

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)

	time.Sleep(5 * time.Second)
	stopAndAssertNoError(t, rec, ctx)
	<-rec.exited

	hasFragments := hasFragmentedMP4Structure(t, rec.outputPath)
	assert.True(t, hasFragments, "MP4 should have fragmented structure")
}

func TestFFmpegRecording_TimestampValidity(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	rec := setupTestRecorder(t, "timestamp-test", nil)

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)

	time.Sleep(3 * time.Second)
	stopAndAssertNoError(t, rec, ctx)
	<-rec.exited

	validateTimestamps(t, rec.outputPath)
}

func TestFFmpegRecording_FileSizeLimit(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	fileSizeGoalMB := 1
	fileSizeGoalMBMaximum := 1.5
	frameRate := 60
	rec := setupTestRecorder(t, "filesizelimit-test", &FFmpegRecordingParams{
		MaxSizeInMB: &fileSizeGoalMB,
		FrameRate:   &frameRate,
	})

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)

	sleepStep := 10
	for sleepTotal := sleepStep; sleepTotal < 300; sleepTotal += sleepStep {
		time.Sleep(time.Duration(sleepStep) * time.Second)
		if !rec.IsRecording(ctx) {
			break
		}
	}

	if rec.IsRecording(ctx) {
		t.Error("recording did not conclude automatically")
		rec.Stop(ctx)
	}
	<-rec.exited

	fileStat, err := os.Stat(rec.outputPath)
	require.NoError(t, err)

	fileSizeMB := float64(fileStat.Size()) / 1024 / 1024
	assert.LessOrEqual(t, fileSizeMB, fileSizeGoalMBMaximum, "file size should not exceed limit significantly")
	t.Logf("File size: %.2f MB, target %d MB, maximum %.2f MB", fileSizeMB, fileSizeGoalMB, fileSizeGoalMBMaximum)
}

func TestFFmpegRecording_DurationLimit(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	maxDuration := 3
	rec := setupTestRecorder(t, "durationlimit-test", &FFmpegRecordingParams{
		MaxDurationInSeconds: &maxDuration,
	})

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)
	time.Sleep(time.Duration(maxDuration*2) * time.Second)

	if rec.IsRecording(ctx) {
		t.Error("recording should have stopped automatically")
		rec.Stop(ctx)
	}
	<-rec.exited

	validateMP4Recording(t, rec.outputPath, videoValidationOpts{
		minDuration: time.Duration(maxDuration*1000-500) * time.Millisecond,
		maxDuration: time.Duration(maxDuration) * time.Second,
	})
}

func TestFFmpegRecording_FrameRateVariety(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	frameRates := []int{1, 5, 10, 20, 24, 25, 30, 60}

	for _, fr := range frameRates {
		t.Run(fmt.Sprintf("FrameRate_%d", fr), func(t *testing.T) {
			rec := setupTestRecorder(t, fmt.Sprintf("framerate-%d-test", fr), &FFmpegRecordingParams{
				FrameRate: &fr,
			})

			ctx := context.Background()
			err := rec.Start(ctx)
			require.NoError(t, err)

			time.Sleep(3 * time.Second)
			stopAndAssertNoError(t, rec, ctx)
			<-rec.exited

			validateMP4Recording(t, rec.outputPath, videoValidationOpts{
				expectedFrameRate:         fr,
				frameRatePercentTolerance: 20,
			})
		})
	}
}

func TestFFmpegRecording_DurationVariety(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	durations := []int{1, 2, 3, 5, 8, 13, 21, 34, 55}

	for i, duration := range durations {
		t.Run(fmt.Sprintf("Duration_%d", duration), func(t *testing.T) {
			rec := setupTestRecorder(t, fmt.Sprintf("durationvariety-%d-test", duration), &FFmpegRecordingParams{
				MaxDurationInSeconds: &duration,
			})

			ctx := context.Background()
			err := rec.Start(ctx)
			require.NoError(t, err)

			time.Sleep(time.Duration(duration+i+1) * time.Second)
			if rec.IsRecording(ctx) {
				t.Error("recording should have automatically concluded")
				stopAndAssertNoError(t, rec, ctx)
			}
			<-rec.exited

			validateMP4Recording(t, rec.outputPath, videoValidationOpts{
				minDuration: time.Duration(duration) * time.Second,
				maxDuration: time.Duration(duration+i+1) * time.Second,
			})
		})
	}
}

func TestFFmpegRecording_ResolutionAccuracy(t *testing.T) {
	display := requireDisplayEnvironmentVariable(t)
	width, height := getScreenResolution(t, display)

	rec := setupTestRecorder(t, "resolution-test", nil)

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)

	time.Sleep(2 * time.Second)
	stopAndAssertNoError(t, rec, ctx)
	<-rec.exited

	validateMP4Recording(t, rec.outputPath, videoValidationOpts{
		expectedWidth:  width,
		expectedHeight: height,
	})
}

func TestFFmpegRecording_GracefulShutdown(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	rec := setupTestRecorder(t, "graceful-shutdown-test", nil)

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)

	time.Sleep(2 * time.Second)

	stopAndAssertNoError(t, rec, ctx)
	if rec.IsRecording(ctx) {
		t.Fatal("recording failed to stop")
	}
	<-rec.exited

	require.FileExists(t, rec.outputPath)
	validateMP4Recording(t, rec.outputPath, videoValidationOpts{
		minDuration: 1 * time.Second,
	})
}

func TestFFmpegRecording_ForceShutdown(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	rec := setupTestRecorder(t, "force-shutdown-test", nil)

	ctx := context.Background()
	err := rec.Start(ctx)
	require.NoError(t, err)

	time.Sleep(2 * time.Second)

	rec.ForceStop(ctx)
	if rec.IsRecording(ctx) {
		t.Fatal("recording failed to stop")
	}
	<-rec.exited

	require.FileExists(t, rec.outputPath)
}

func TestFFmpegScreenshot_FullScreen(t *testing.T) {
	display := requireDisplayEnvironmentVariable(t)
	width, height := getScreenResolution(t, display)

	args := []string{
		"-f", "x11grab",
		"-video_size", fmt.Sprintf("%dx%d", width, height),
		"-i", display,
		"-vframes", "1",
		"-f", "image2pipe",
		"-vcodec", "png",
		"-",
	}

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	cmd := exec.CommandContext(ctx, getFFmpegBinaryPath(t), args...)
	output, err := cmd.Output()
	require.NoError(t, err, "ffmpeg screenshot command should succeed")

	validatePNGScreenshot(t, output, width, height)
}

func TestFFmpegScreenshot_CroppedRegion(t *testing.T) {
	display := requireDisplayEnvironmentVariable(t)
	width, height := getScreenResolution(t, display)
	cropX, cropY, cropWidth, cropHeight := 100, 100, 200, 200

	if cropX+cropWidth > width || cropY+cropHeight > height {
		t.Errorf("Screen too small for crop test (need at least %dx%d, have %dx%d)",
			cropX+cropWidth, cropY+cropHeight, width, height)
	}

	args := []string{
		"-f", "x11grab",
		"-video_size", fmt.Sprintf("%dx%d", width, height),
		"-i", display,
		"-vframes", "1",
		"-vf", fmt.Sprintf("crop=%d:%d:%d:%d", cropWidth, cropHeight, cropX, cropY),
		"-f", "image2pipe",
		"-vcodec", "png",
		"-",
	}

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	cmd := exec.CommandContext(ctx, getFFmpegBinaryPath(t), args...)
	output, err := cmd.Output()
	require.NoError(t, err, "cropped screenshot should succeed")
	require.NotEmpty(t, output, "should produce screenshot output")

	validatePNGScreenshot(t, output, cropWidth, cropHeight)
}

func TestFFmpegError_InvalidDisplay(t *testing.T) {
	invalidDisplay := 999
	outputDir := t.TempDir()

	rec := setupTestRecorder(t, "invalid-display-test", &FFmpegRecordingParams{
		DisplayNum: &invalidDisplay,
		OutputDir:  &outputDir,
	})

	ctx := context.Background()
	err := rec.Start(ctx)

	if err == nil {
		select {
		case <-rec.exited:
			assert.NotEqual(t, 0, rec.exitCode, "should exit with non-zero code for invalid display")
		case <-time.After(2 * time.Second):
			rec.ForceStop(ctx)
			t.Fatal("recording should fail quickly with invalid display")
		}
	}
}

func TestFFmpegError_UnsupportedCodec(t *testing.T) {
	display := requireDisplayEnvironmentVariable(t)

	outputPath := filepath.Join(t.TempDir(), "unsupported-codec-test.mp4")

	args := []string{
		"-f", "x11grab",
		"-i", display,
		"-c:v", "nonexistent_codec_xyz",
		"-y",
		outputPath,
	}

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	cmd := exec.CommandContext(ctx, getFFmpegBinaryPath(t), args...)
	err := cmd.Run()
	assert.Error(t, err, "should fail with unsupported codec")
}

func TestFFmpegError_InvalidFlag(t *testing.T) {
	t.Parallel()

	cmd := exec.Command(getFFmpegBinaryPath(t), "-invalid_flag_xyz", "-version")
	err := cmd.Run()

	assert.Error(t, err, "command should exit with an error")
	assert.ErrorContains(t, err, "exit status 8")
}

func TestFFmpegError_MissingOutputDirectory(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	invalidDir := "/nonexistent/directory"
	rec := setupTestRecorder(t, "missing-dir-test", &FFmpegRecordingParams{
		OutputDir: &invalidDir,
	})

	ctx := context.Background()
	err := rec.Start(ctx)

	if err == nil {
		select {
		case <-rec.exited:
			assert.Error(t, err, "should have an error from missing output directory")
			assert.Equal(t, 254, rec.exitCode, "should fail with exit code for missing output directory")
		case <-time.After(2 * time.Second):
			rec.ForceStop(ctx)
			t.Fatal("should have stopped due to invalid output directory")
		}
	}
}

func TestFFmpegError_PermissionDenied(t *testing.T) {
	if os.Getuid() == 0 {
		t.Skip("Skipping permission test when running as root")
	}

	requireDisplayEnvironmentVariable(t)

	readOnlyDir := t.TempDir()
	assert.NoError(t, os.Chmod(readOnlyDir, 0400), "error changing directory mode")
	rec := setupTestRecorder(t, "permission-test", &FFmpegRecordingParams{
		OutputDir: &readOnlyDir,
	})

	ctx := context.Background()
	err := rec.Start(ctx)

	if err == nil {
		select {
		case <-rec.exited:
			assert.Error(t, err, "should have an error from insufficient permissions")
			assert.Equal(t, 243, rec.exitCode, "should fail with exit code for insufficient permissions")
		case <-time.After(2 * time.Second):
			rec.ForceStop(ctx)
			t.Fatal("process should have exited due to insufficient permissions")
		}
	}
}

func TestFFmpegError_RecordingRecovery(t *testing.T) {
	requireDisplayEnvironmentVariable(t)

	invalidDisplay := 999
	outputDir := t.TempDir()
	rec1 := setupTestRecorder(t, "recovery-fail-test", &FFmpegRecordingParams{
		DisplayNum: &invalidDisplay,
		OutputDir:  &outputDir,
	})

	ctx := context.Background()
	rec1.Start(ctx)

	select {
	case <-rec1.exited:
	case <-time.After(2 * time.Second):
		rec1.ForceStop(ctx)
	}

	rec2 := setupTestRecorder(t, "recovery-success-test", nil)
	err := rec2.Start(ctx)
	require.NoError(t, err, "should be able to start new recording after failure")

	time.Sleep(2 * time.Second)
	rec2.Stop(ctx)
	<-rec2.exited

	require.FileExists(t, rec2.outputPath, "recovery recording should exist")
}
