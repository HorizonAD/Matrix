cd ./protocol/raw
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./
./protoc -I./ --cpp_out=../ ./can.proto ./common.proto ./sensor.proto ./meta_data.proto ./meta.proto