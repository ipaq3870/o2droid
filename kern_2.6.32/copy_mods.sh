rm -rf ../O2_modules32/*
find ../modules32 -name '*.ko' -exec cp {} ../O2_modules32/  \;
find . -name '*.ko' -exec cp {} ../O2_modules32/  \;
cd ../O2_modules32/ 
tar cf ../O2_kos32.tar *
