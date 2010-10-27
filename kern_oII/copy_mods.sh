rm -rf ../O2_modules/*
find ../modules -name '*.ko' -exec cp {} ../O2_modules/  \;
find . -name '*.ko' -exec cp {} ../O2_modules/  \;
cd ../O2_modules/ 
tar cf ../O2_kos.tar *
