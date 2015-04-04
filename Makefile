TARGET := spidy1
MCU := atmega32u4
CFLAGS := -g -O2 -Wall -std=gnu11 -mmcu=$(MCU) -DF_CPU=16000000L -ffunction-sections -fdata-sections
LDFLAGS := -Wl,-Map,$(TARGET).map -Wl,--gc-sections
PROGPORT ?= /dev/ttyACM0
PROGFLAGS := -p m32u4 -P $(PROGPORT) -c avr109 -u

COMPILE = avr-gcc -c -o $@ $(CFLAGS) $<
LINK = avr-gcc -o $@ $(CFLAGS) $^ $(LDFLAGS)
INSTALL = avrdude $(PROGFLAGS) -U flash:w:$<:s

.PHONY: all clean
all: srec
	@:
clean:
	-$(RM) -f *.o *.srec *.lst *.map $(TARGET)

$(TARGET): main.o servo.o
	$(LINK)

%.o: %.c common.h servo.h
	$(COMPILE)

.PHONY: listing srec
listing: $(TARGET).lst
srec: $(TARGET).srec
	@:
%.lst: %
	avr-objdump -h -S $< > $@
%.srec: %
	avr-objcopy -O srec -R .eeprom $< $@
	avr-size --mcu=$(MCU) -C --format=avr $<

.PHONY: install readback
install: $(TARGET).srec
	$(INSTALL)
readback: readback.srec
	@:
readback.srec:
	avrdude $(PROGFLAGS) -U flash:r:$@:s
