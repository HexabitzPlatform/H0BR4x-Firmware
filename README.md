# H0BR40 Module #

**H0BR40 is a 3-axis initial measurement unit (IMU) combined with a 3-axis digital compass module** based on STM32F0 MCU, LSM6DS3 IMU and LSM303AGR compass. It is part of Hexabitz modular prototyping system.

Module web page: https://hexabitz.com/product/imu-and-digital-compass-h0br40/

- Check the *References* tab in module page for links to documentation, design files, source code and examples.

- Check the *Quickstart Guide* tab in module page for quick introduction on using this module.

===============================================

## About Hexabitz ##

Hexabitz is a new kind of electronic prototyping platforms with game-changing modularity and a hint of biomimicry. Hexabitz modules connect with each other using a novel edge-soldering system and allow you to build tidy, compact and completely re-configurable electronic boards. **Learn more about Hexabitz [here](https://www.hexabitz.com/)**.

===============================================

## Submodules ##

- Bitz Operating System - [BOS](https://bitbucket.org/hexabitz/bos)
- [Thirdparty](https://bitbucket.org/hexabitz/thirdparty)

See also other module repositories [here](https://bitbucket.org/hexabitz/)

===============================================

## How do I get set up? ##

### If you want to load a precompiled HEX file ###

1- Navigate to *Compiled* folder and load the appropriate HEX file using any firmware update method described [here](https://hackaday.io/project/76446-hexabitz-modular-electronics-for-real/log/137477-how-to-update-module-firmware).

### If you want to compile the code: ###

1- If you don't have uVision installed already, download Keil uVision MDK toolchain from [here](http://www2.keil.com/mdk5/uvision/). Get your free [license](http://www.keil.com/) for STM32F0 MCUs!

2- Download or clone this repository source code and open the uVion projext in MDK-ARM folder (it has .uvprojx extension).

3- If you are loading a single module, simply compile the code and load it to module MCU via one of the firmware update methods explaind [here](https://hackaday.io/project/76446-hexabitz-modular-electronics-for-real/log/137477-how-to-update-module-firmware).

4- If you are loading multiple modules of the same type (connected in an array) and you want them to have unique firmware, then manually modify the module ID in Options for Target >> C/C++ >> Preprocessor Symbols >> Define >> _module=x (where x is the module ID) and in Output >> Name of Executable. Recompile the project and load each module according to its ID. You can also create multiple targets as explained in the firmware update [guide]().

You can also compile using GCC on your favorite toolchain.

### How do I test? ###

1- If code is loaded correctly, you should see one or few indicator LED blinks when you power-cycle.

2- Use [RealTerm](https://sourceforge.net/projects/realterm/) or any other terminal emulator with a USB-to-USRT cable to access the module CLI. Learn about using the CLI [here](https://hackaday.io/project/76446-hexabitz-modular-electronics-for-real/log/137487-using-the-command-line-interface-cli).

3- Check available CLI commands by typing *help* or use the module factsheet. Make sure the factsheet BOS version number (at the footer) matches the source code version you have.

### How do I update the source code for an old project? ###

1- If your project follows portability guidelines, then just keep all files in the *User* folder and replace all other folders with the newer source code.

### Where should I add my code? ###

To preseve maximum code portability between projects, we advise you to:

1- Keep all your custom source and header files in the *User* folder.

2- Add your code to the *FrontEndTask()* in *main.c* and add other custom functions there including custom interrupt callbacks.

3- Add any external header files you want to include to the *project.h* file.

4- Add any topology header files to *User* folder and include them in *project.h*.

===============================================

## FAQ ##

Check out the *FAQ* tab in module page or our general [FAQ](https://www.hexabitz.com/faq/) section on the website. Feel free to [contact](info@hexabitz.com) us about any questions or feedback!

===============================================

## Whom do I talk to? ##

* [Email](info@hexabitz.com) us for any inquiries
* Or connect with us on [Hackaday.io](https://hackaday.io/Hexabitz), [Twitter](https://twitter.com/HexabitzInc) or [Facebook](https://www.facebook.com/HexabitzInc/)!

## How do I contribute? ##

* We welcome any and all help you can provide with testing, bug fixing and adding new features! Feel free to contact us and share what's going on in your mind.
* Please send us a pull request if you have some useful code to add!

===============================================

## License ##
This software / firmware is released with [MIT license](https://opensource.org/licenses/MIT). This means you are free to use it in your own projects/products for personal or commercial applications. You are not required to open-source your projects/products as a result of using Hexabitz code inside it.

To our best knowledge, all third-party components currently included with Hexabitz software follow similar licenses (MIT, modified GPL, etc.). We will do our best to not include third-party components that require licensing or have restricted open-source terms (i.e., forcing you to open-source your project). There is no guarantee, however, that this does not happen. If we ever include a software component that requires buying a license or one that forces restrictive, open-source terms, we will mention this clearly. We advise you to verify the license of each third-party component with its vendor. 

## Disclaimer ##
HEXABITZ SOFTWARE AND HARDWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE AND HARDWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE AND HARDWARE.
