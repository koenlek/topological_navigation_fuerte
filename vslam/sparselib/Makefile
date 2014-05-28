all: SparseLib

TARBALL = build/sparselib_1_7.zip
TARBALL_URL = http://pr.willowgarage.com/downloads/sparselib_1_7.zip
UNPACK_CMD = unzip -qq -o
# patching doesn't work with 2-level source directory
# TARBALL_PATCH = sparselib.patch
SOURCE_DIR = build/SparseLib++/1.7
MD5SUM_FILE = sparselib_1_7.zip.md5sum
include $(shell rospack find mk)/download_unpack_build.mk

SparseLib: $(SOURCE_DIR)/unpacked
	-rm -rf lib include
	mkdir -p lib
	mkdir -p include/SparseLib
	cp sparselib.patch $(SOURCE_DIR)
	cd $(SOURCE_DIR) && patch -p0 < sparselib.patch && make sp
	cp $(SOURCE_DIR)/include/*.h include/SparseLib
	cp $(SOURCE_DIR)/mv/include/*.h include/SparseLib
	cp $(SOURCE_DIR)/lib/*.a lib
	cp cg.h include/SparseLib
	touch SparseLib
clean:
	rm -rf lib include $(SOURCE_DIR)
wipe: clean
	-rm -rf build $(TARBALL) SparseLib
