# 
## Instalation:
* Install VScode and pico-sdk extension
* Create a project from examples
# export PICO_SDK_PATH=~/.pico-sdk/sdk/2.1.1
```
export PICO_SDK_PATH=~/.pico-sdk
export CMAKE_GENERATOR=Ninja
mkdir build && cd build
cmake ..
$PICO_SDK_PATH/ninja/v1.12.1/ninja -C ./
$PICO_SDK_PATH/picotool/2.1.1/picotool/picotool load ./hello_usb.uf2 -fx
```


