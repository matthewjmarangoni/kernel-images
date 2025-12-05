SHELL := /bin/bash
.PHONY: oapi-generate build dev test test-ffmpeg-integration-headful test-ffmpeg-integration-headless clean

BIN_DIR ?= $(CURDIR)/bin
OAPI_CODEGEN ?= $(BIN_DIR)/oapi-codegen
RECORDING_DIR ?= $(CURDIR)/recordings

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(RECORDING_DIR):
	mkdir -p $(RECORDING_DIR)

$(OAPI_CODEGEN): | $(BIN_DIR)
	GOBIN=$(BIN_DIR) go install github.com/oapi-codegen/oapi-codegen/v2/cmd/oapi-codegen@latest

# Generate Go code from the OpenAPI spec
# 1. Convert 3.1 → 3.0 since oapi-codegen doesn't support 3.1 yet (https://github.com/oapi-codegen/oapi-codegen/issues/373)
# 2. Run oapi-codegen with our config
# 3. go mod tidy to pull deps
oapi-generate: $(OAPI_CODEGEN)
	pnpm i -g @apiture/openapi-down-convert
	openapi-down-convert --input openapi.yaml --output openapi-3.0.yaml
	$(OAPI_CODEGEN) -config ./oapi-codegen.yaml ./openapi-3.0.yaml
	@echo "Fixing oapi-codegen issue https://github.com/oapi-codegen/oapi-codegen/issues/1764..."
	go run ./scripts/oapi/patch_sse_methods.go -file ./lib/oapi/oapi.go -expected-replacements 3
	go fmt ./lib/oapi/oapi.go
	go mod tidy

build: | $(BIN_DIR)
	go build -o $(BIN_DIR)/api ./cmd/api

dev: build $(RECORDING_DIR)
	OUTPUT_DIR=$(RECORDING_DIR) DISPLAY_NUM=$(DISPLAY_NUM) ./bin/api

test:
	go vet ./...
	go test -v -race ./...

test-ffmpeg-integration-headless:
	@echo "Running FFmpeg integration tests in headless container ..."
	@IMAGE_HEADLESS="$(or $(IMAGE_HEADLESS),onkernel/chromium-headless-test:latest)"; \
	IMAGE_HEADLESS=$$IMAGE_HEADLESS DISPLAY_NUM=$(DISPLAY_NUM) ./lib/recorder/test-ffmpeg-integration.sh

test-ffmpeg-integration-headful:
	@echo "Running FFmpeg integration tests in headful container ..."
	@IMAGE_HEADFUL="$(or $(IMAGE_HEADFUL),onkernel/chromium-headful-test:latest)"; \
	IMAGE_HEADFUL=$$IMAGE_HEADFUL DISPLAY_NUM=$(DISPLAY_NUM) ./lib/recorder/test-ffmpeg-integration.sh

clean:
	@rm -rf $(BIN_DIR)
	@rm -f openapi-3.0.yaml
	@echo "Clean complete"

DISPLAY_NUM := $(shell \
	if [ "$$(uname)" = "Linux" ]; then \
		echo "1"; \
	elif [ "$$(uname)" = "Darwin" ]; then \
		ffmpeg -f avfoundation -list_devices true -i "" 2>&1 | grep "Capture screen" | head -1 | sed 's/.*\[\([0-9]*\)\].*/\1/' 2>/dev/null || echo "2"; \
	else \
		echo "0"; \
	fi)
