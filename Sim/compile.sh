Program_name="test"
make
./B2H_CONVERTER ${Program_name}.bin ${Program_name}.hex
cp ${Program_name}.hex ../${Program_name}.hex