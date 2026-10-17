# fldigi-mt63-core -- plain make, C99, no dependencies besides libm. MT63 vendors Pawel
# Jalocha's own DSP (src/mt63_dsp.c) instead of fldigi-dsp-core (wave-owl R-920.4).
#   make            static + shared library into $(BUILD)
#   make test       build and run the C tests (receiver, transmitter, fldigi goldens)
#   make sanitize   the C tests under AddressSanitizer + UndefinedBehaviorSanitizer
#   make bench      CPU and memory benchmark (per submode, receiver and transmitter)
#   make BUILD=/abs/dir shared   out-of-tree build (used by wave-owl)
BUILD ?= build
CC ?= cc
# -ffp-contract=on: fuse a*b+c within one expression, as clang and gcc do by default and as
# the fldigi 4.2.13 reference build did.
CFLAGS ?= -O3
CORE_FLAGS = -std=c99 -Wall -Wextra -Werror -pedantic -ffp-contract=on -fPIC -Iinclude
LDLIBS = -lm
# Tests rebuild the golden noise exactly as the golden script does: no contraction there.
TEST_FLAGS = -std=c99 -Wall -Wextra -Werror -pedantic -ffp-contract=off -Iinclude -Isrc

UNAME := $(shell uname)
ifeq ($(UNAME),Darwin)
  SHLIB = libfldigi_mt63.dylib
  SHFLAGS = -dynamiclib -install_name @rpath/libfldigi_mt63.dylib
else
  SHLIB = libfldigi_mt63.so
  SHFLAGS = -shared -Wl,-soname,libfldigi_mt63.so
endif

HDRS = include/fldigi_mt63.h src/mt63_internal.h
OBJ = $(BUILD)/mt63_common.o $(BUILD)/mt63_dsp.o $(BUILD)/mt63_tables.o \
      $(BUILD)/mt63_rx.o $(BUILD)/mt63_tx.o

all: static shared
static: $(BUILD)/libfldigi_mt63.a
shared: $(BUILD)/$(SHLIB)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.c $(HDRS) | $(BUILD)
	$(CC) $(CORE_FLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/libfldigi_mt63.a: $(OBJ)
	rm -f $@
	ar rcs $@ $^

$(BUILD)/$(SHLIB): $(OBJ)
	rm -f $@
	$(CC) $(SHFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD)/test_mt63: tests/test_mt63.c tests/test_util.h $(BUILD)/libfldigi_mt63.a
	$(CC) $(TEST_FLAGS) $(CFLAGS) $< $(BUILD)/libfldigi_mt63.a -o $@ $(LDLIBS)

$(BUILD)/test_mt63_fldigi: tests/test_mt63_fldigi.c tests/test_util.h $(BUILD)/libfldigi_mt63.a
	$(CC) $(TEST_FLAGS) $(CFLAGS) $< $(BUILD)/libfldigi_mt63.a -o $@ $(LDLIBS)

test: $(BUILD)/test_mt63 $(BUILD)/test_mt63_fldigi
	$(BUILD)/test_mt63
	$(BUILD)/test_mt63_fldigi tests/golden

SAN_FLAGS = -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all

sanitize:
	$(MAKE) --no-print-directory BUILD=$(BUILD)/san CFLAGS="$(SAN_FLAGS) -DMT63_NOT_REFERENCE" \
		LDLIBS="$(LDLIBS) $(SAN_FLAGS)" test

$(BUILD)/bench_mt63: tests/bench_mt63.c tests/test_util.h $(BUILD)/libfldigi_mt63.a
	$(CC) $(TEST_FLAGS) $(CFLAGS) $< $(BUILD)/libfldigi_mt63.a -o $@ $(LDLIBS)

bench: $(BUILD)/bench_mt63
	$(BUILD)/bench_mt63

clean:
	rm -rf $(BUILD)

.PHONY: all static shared test sanitize bench clean
