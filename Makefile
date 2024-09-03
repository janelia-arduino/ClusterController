.PHONY: clean
clean:
	rm -rf .pio

.PHONY: pico-firmware
pico-firmware: clean
	pio run -e pico

.PHONY: pico-upload
pico-upload: clean
	pio run -e pico --target upload

.PHONY: monitor
monitor:
	pio device monitor

.PHONY: udev-rules
udev-rules:
	curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
