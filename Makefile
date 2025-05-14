.PHONY: clean
clean:
	rm -rf .pio

.PHONY: firmware
firmware:
	pio run -e pico

.PHONY: firmware-clean
firmware-clean: clean
	pio run -e pico

.PHONY: upload
upload:
	pio run -e pico --target upload

.PHONY: upload-clean
upload-clean: clean
	pio run -e pico --target upload

PORTS := $(wildcard /dev/ttyACM*)
.PHONY: upload-all
upload-all:
	@for port in $(PORTS); do \
		echo "Uploading to $$port"; \
		pio run -e pico --target upload; \
	done

.PHONY: monitor
monitor:
	pio device monitor

.PHONY: udev-rules
udev-rules:
	curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
