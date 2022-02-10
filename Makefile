TARGET := emulator

BUILD_DIR := build
SRC_DIR := src
INC_DIR := inc

SRCS = $(shell find $(SRC_DIR) -name *.cpp)
OBJS = $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS = $(OBJS:.o=.d)
LIBS := 

INC_FLAGS := $(addprefix -I,$(INC_DIR))
CXX_FLAGS := -g -std=c++17 -Wall -Wextra $(INC_FLAGS) -MMD -MP
LD_FLAGS := -g $(addprefix -l,$(LIBS))

$(BUILD_DIR)/$(TARGET): $(OBJS)
	$(CXX) $(LD_FLAGS) $(OBJS) -o $@

$(BUILD_DIR)/%.cpp.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXX_FLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
