all: g2o

SOURCE_DIR = build/g2o
SVN_DIR = $(SOURCE_DIR)
TOP_DIR = $(PWD)

# tarball download
#TARBALL = build/g2o.tar.gz
#TARBALL_URL = http://pr.willowgarage.com/downloads/g2o.tar.gz
#TARBALL_URL = http://vault.willowgarage.com/wgdata1/vol1/g2o/g2o.tar.gz
#TARBALL_URL = file:///home/konolige/devel/g2o.tar.gz
#TARBALL_PATCH = makefile.patch
#MD5SUM_FILE = g2o.tar.gz.md5sum
#include $(shell rospack find mk)/download_unpack_build.mk

# svn download
SVN_URL =  https://svn.openslam.org/data/svn/g2o/trunk
#SVN_REVISION = -r233
#SVN_CMDLINE = svn --no-auth-cache --trust-server-cert --non-interactive
SVN_CMDLINE = svn --non-interactive --config-dir $(TOP_DIR)/svnconf
include $(shell rospack find mk)/svn_checkout.mk

#g2o: $(SOURCE_DIR)/unpacked
g2o: $(SOURCE_DIR)
	-rm -rf lib include bin
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && cmake ../ -DCMAKE_INSTALL_PREFIX=$(TOP_DIR) && make install
	touch g2o

clean:
	rm -rf lib bin include $(SOURCE_DIR)
wipe: clean
	-rm -rf build g2o
