LEVEL = ../

include $(LEVEL)makeinclude

#########################

LOCAL_FILES = EVILib.o\
	EVILib.a

#########################

ALL = $(LOCAL_FILES)

main: $(ALL)

all: $(ALL)

#########################

EVILib.o: EVILib.c++ EVILib.h
	@echo Compiling $@
	$(CC) $(OPTIONS) $(INCLUDES) -o $@ -c $<

EVILib.a : EVILib.o
	@echo == making $@ ==
	$(AR) -rc $@ $<
	@echo

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
