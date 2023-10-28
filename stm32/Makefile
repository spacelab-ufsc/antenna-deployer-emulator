FLASH=st-flash

.PHONY: all
all: 
	$(MAKE) -C libopencm3
	$(MAKE) -C antenna-deployer-emulator

.PHONY: flash
flash:
	$(FLASH) --reset write antenna-deployer-emulator/antenna-deployer-emulator.bin 0x08000000

