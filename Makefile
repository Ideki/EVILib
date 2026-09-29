LEVEL =

include makeinclude

#########################

LIB = EVILib.a
LIB_DIR = src

SAMPLES = evilib
SAMPLES_DIR = samples

#########################

main: $(LIB)

all: $(LIB)\
        $(SAMPLES)

#########################

EVILib.a :
	cd $(LIB_DIR); $(MAKE) $(MFLAGS);

#########################

evilib : $(LIB)
	cd $(SAMPLES_DIR); $(MAKE) $(MFLAGS);

#########################

clean:
	-$(RM) *~ 

rclean:
	cd $(LIB_DIR); $(MAKE) rclean;
	cd $(SAMPLES_DIR); $(MAKE) rclean;
