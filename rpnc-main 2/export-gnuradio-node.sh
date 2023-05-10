export UHD_INSTALL_DIR=/home/lzyou/uhd
export PATH=$PATH:$UHD_INSTALL_DIR/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$UHD_INSTALL_DIR/lib
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$UHD_INSTALL_DIR/lib/pkgconfig
export PYTHONPATH=$PYTHONPATH:$UHD_INSTALL_DIR/lib/python2.7/dist-packages

export GNURADIO_INSTALL_PATH=/home/user/gnuradio/gnuradio-3.7.8/install
export PATH=$PATH:$GNURADIO_INSTALL_PATH/bin
export PYTHONPATH=$PYTHONPATH:$GNURADIO_INSTALL_PATH/lib/python2.7/dist-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GNURADIO_INSTALL_PATH/lib
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$GNURADIO_INSTALL_PATH/lib/pkgconfig

export RPNC_INSTALL_DIR=/home/user/rpnc-chenjian/rpnc-release-20160106/upyou-rpnc-release-7621b9ddcd85-gr3.7.x/install
export PYTHONPATH=$PYTHONPATH:$RPNC_INSTALL_DIR/lib/python2.7/dist-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RPNC_INSTALL_DIR/lib
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$RPNC_INSTALL_DIR/lib/pkgconfig

sudo sysctl -w net.core.wmem_max=1048576
sudo sysctl -w net.core.rmem_max=50000000
