
PROJECTS = ssd1306test

all clean:
	@for P in $(PROJECTS); do $(MAKE) -f $$P.mk $@ || exit 1 ; done


