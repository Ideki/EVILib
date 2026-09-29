LEVEL = ../

include $(LEVEL)makeinclude

#########################

LOCAL_FILES = evilib.o evilib

#########################

ALL = $(LOCAL_FILES)

main: $(ALL)

all: $(ALL)

#########################

evilib.o: evilib.c++ evilib.h
	@echo Compiling $@
	$(CC) $(OPTIONS) $(INCLUDES) -o $@ -c $<

evilib: evilib.o
	@echo Making $@
	$(CC) -g $(OPTIONS) evilib.o $(LL)EVILib.a $(LIBDIRS) $(LIBS) -o $@

#########################

clean:
	-$(RM) *~

rclean:
	$(MAKE) clean
	@for files in $(LOCAL_FILES); do\
		($(RM) $$files);\
	done

clock:
	$(TOUCH) *
