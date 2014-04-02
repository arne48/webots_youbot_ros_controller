OSTYPE ?= $(shell uname)

ifeq ($(OSTYPE),linux)
OSTYPE=Linux
endif

ifneq ($(findstring MINGW,$(OSTYPE)),)
OSTYPE=Windows
endif

ifeq ($(OSTYPE),msys)
OSTYPE=Windows
endif

ifeq ($(OSTYPE),Window)
ROSMAKE=echo 'ROS controllers are not supported under Windows'
endif

ifeq ($(OSTYPE),Darwin)
ROSMAKE=echo 'ROS controllers not compiled under Mac OS X'
endif

ifeq ($(OSTYPE),Linux)
ROSMAKE=rosmake
endif

ROS_PACKAGE_PATH:=${ROS_PACKAGE_PATH}

compile: youbot

youbot:
	@cd ../; $(ROSMAKE)
	@cp ../bin/youbot ../controllers/youbot/
	@rm -rf ../../build

clean:
	@rm -rf ../build ../controllers/youbot/youbot

