#!/bin/bash
CURRENTDIR=$(dirname $(readlink -f $0))
CONF_FILE=Arena_SDK.conf

###############################################################################
echo
echo "Arena SDK configuration script"
echo
echo "Removing existing $CONF_FILE"
echo
###############################################################################

sudo rm -f /etc/ld.so.conf.d/$CONF_FILE

###############################################################################
echo "Adding the following Arena SDK library paths to /etc/ld.so.conf.d/$CONF_FILE:"
echo "$CURRENTDIR/lib64"
sh -c "echo $CURRENTDIR/lib64 > /etc/ld.so.conf.d/$CONF_FILE"

echo "$CURRENTDIR/GenICam/library/lib/Linux64_x64"
sh -c "echo $CURRENTDIR/GenICam/library/lib/Linux64_x64 >> /etc/ld.so.conf.d/$CONF_FILE"

echo "$CURRENTDIR/ffmpeg"
sh -c "echo $CURRENTDIR/ffmpeg >> /etc/ld.so.conf.d/$CONF_FILE"

###############################################################################
echo
echo "Please remember to install these packages before proceeding:"
echo "- g++ 5 or higher"
echo "- make"
echo

###############################################################################

ldconfig
