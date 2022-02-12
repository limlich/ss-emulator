TARGET := emulator
DEBUG_ENABLED = 1

BUILD_DIR := build
SRC_DIR := src
INC_DIR := inc

SRCS = $(shell find $(SRC_DIR) -name *.cpp)
OBJS = $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS = $(OBJS:.o=.d)
LIBS := 

DEBUG_FLAGS = -g

INC_FLAGS := $(addprefix -I,$(INC_DIR))
CXX_FLAGS := -std=c++17 -Wall -Wextra $(INC_FLAGS) -MMD -MP
LD_FLAGS := $(addprefix -l,$(LIBS))
ifeq ($(DEBUG_ENABLED),1)
CXX_FLAGS += $(DEBUG_FLAGS)
LD_FLAGS += $(DEBUG_FLAGS)
endif

$(BUILD_DIR)/$(TARGET): $(OBJS)
	$(CXX) $(LD_FLAGS) $(OBJS) -o $@

$(BUILD_DIR)/%.cpp.o: %.cpp Makefile
	mkdir -p $(dir $@)
	$(CXX) $(CXX_FLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
