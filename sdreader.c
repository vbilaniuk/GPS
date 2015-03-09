/* sdreader - allows you to download files off an SD card attached to an Arduino Uno without having to physically remove the card or reprogram the Arduino.  Just make sure the Arduino has some basic support programmed into your sketch, then plug in the USB cable and go.
 *
 * Mac version.
 *
 * Based on a combination of http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 * and https://developer.apple.com/library/mac/documentation/DeviceDrivers/Conceptual/WorkingWSerial/WWSerial_SerialDevs/SerialDevices.html
 * and my own additions.
 *
 * Usage:
 * sdreader                     lists help
 * sdreader -h                  lists help
 * sdreader --help              lists help
 * sdreader l                   lists available USB ports
 * sdreader portname            lists files on the SD card at portname
 * sdreader portname filename   dumps contents of filename at portname
 *
 * HOW TO SET UP THE ARDUINO
 * Set it to open a serial connection running at 230400 bps.  Use CoolTerm if you need to test it because the terminal provided with the Arduino IDE does not support this speed.
 * This is currently intended to be used with an Arduino Uno (not a Leonardo) with any basic SD card reader attached.
 * Device must accept the following commands and return the following output:
 * 1.  receive single char d or D and send back a directory listing of the root directory on the SD card
 * 2.  receive single char f or F followed by a string filename and send back a file dump of filename
 * Note that it is intended that all files are stored in the root directory.  I have not tested creating subdirectories for
 * the simple reason that I do not have enough memory on my Arduino for that code.  I am assuming only basic support on the Arduino side.
 * I will include my Arduino code separately from this.  Copy/paste into an existing project.
 *
 * You can use this program to redirect output to a text file as follows:
 * sdreader portname filename > fileToSaveOnHardDrive.txt
 * Or you can try the accompanying java program I'm making.  It'll have a GUI and the ability to store to a database.  In fact this was the whole reason I wrote this - so I could build my own GUI that stores the files in the format I want and the location I want.  Otherwise it's perfectly fine to use something like CoolTerm to save the dumps.  I just wrote this C program to avoid the nastiness of trying to set java up to be able to talk to hardware and not break things that depend on my java installation, such as the Arduino IDE.
 *
 * Windows version to come.
 *
 * I do not take any responsibility for this program.  Use at your own risk.  
 *
 * I have tested it with my own Arduino Uno running an Adafruit Ultimate GPS and it works great for me, but of course it would since I wrote it.
 *
 * Future additions:
 * More normal funtionality such as ability to format/delete.  It will be up to the person doing the project to decide how much to support on the hardware side.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <paths.h>
#include <termios.h>
#include <sysexits.h>
#include <sys/param.h>
#include <sys/select.h>
#include <sys/time.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>
#include <time.h>

#define MAXSTRINGLEN 1000

static kern_return_t GetDevices(io_iterator_t *matchingServices);
static kern_return_t ListDevicePaths(io_iterator_t serialPortIterator, char *deviceFilePath, CFIndex maxPathSize);
static int OpenSerialPort(const char *deviceFilePath, int speed);

int main(int argc, char *argv[])
{
    long int n = 0;
    int portHandle = -1;
    kern_return_t kernResult;
    io_iterator_t serialPortIterator;
    char deviceFilePath[MAXPATHLEN];
    fd_set fdset; // make a file descriptor set
    FD_ZERO (&fdset); // init it
    char buf[MAXSTRINGLEN];
    char message;
    struct timeval timeout; // timeout value for the select function
    timeout.tv_sec = 1; // set to 1 second
    
    if (argc == 1 || (argc == 2 && strcmp("--help", argv[1]) == 0) || (argc == 2 && strcmp("-h", argv[1]) == 0) || argc > 3 )
    {
        // print help
        printf("\nsdreader: Reads files from a properly equipped and programmed Arduino.\n");
        printf("Options:\n");
        printf("[l] lists ports\n[portname] lists files on port portname\n[portname filename] downloads filename from portname.\n");
        return EX_OK;
    }

    if (argc == 2 && strcmp(argv[1], "l") == 0)
    {
        // user asked for a port listing
        kernResult = GetDevices(&serialPortIterator);
        printf("Devices on this system:\n");
        kernResult = ListDevicePaths(serialPortIterator, deviceFilePath, sizeof(deviceFilePath));
        
        IOObjectRelease(serialPortIterator);    // Release the iterator.
        return EX_OK;
    }
    
    if (argc == 2 || argc == 3)
    {
        // if this code is being executed, it means the user entered a port name
        // open the port and set up the file descriptor set
        portHandle = OpenSerialPort(argv[1], B230400);
        if (portHandle == -1)
        {
            printf("Error opening serial port.\n");
            return EX_IOERR;
        }
        if (argc == 2) printf("Getting file list from SD card...\n");
        if (argc == 3) printf("Getting file from SD card...\n");
        sleep(2); //a delay of 2 seconds is required to give the Arduino time to boot since it's wired to reboot every time you open a serial connection to it.  This might fail with a Leonardo, which is why I'm specifically excluding it.  I'll probably deal with this later since I actually have a Leonardo...
        FD_SET (portHandle, &fdset); // add to file descriptor set.
    }
    
    if (argc == 2 && portHandle != -1 && strcmp(argv[1], "l") != 0)
    {
        // user asked for a file listing
        // we're going to use select to only read from the file handle when there's data available
        message = 'd';
        n = write(portHandle, &message, sizeof(message));
        if (n == -1)
        {
            printf("Error writing to serial port %s - %s(%d).\n", deviceFilePath, strerror(errno), errno);
            return EX_IOERR;
        }
        
        while (select (FD_SETSIZE, &fdset, NULL, NULL, &timeout))
        {
            n = read(portHandle, &buf, sizeof(buf)-1);
            if (n == -1)
            {
                printf("Error reading from serial port %s - %s(%d).\n", deviceFilePath, strerror(errno), errno);
                return EX_IOERR;
            }
            buf[n] = '\0';
            printf("%s", buf);
        }
    }
    
    if (argc == 3 && portHandle != -1)
    {
        // user has asked for a file dump
        sleep(2);
        message = 'f';
        n = write(portHandle, &message, sizeof(message));
        if (n == -1)
        {
            printf("Error writing to serial port %s - %s(%d).\n", deviceFilePath, strerror(errno), errno);
            return EX_IOERR;
        }
        usleep(5000); // delay for Arduino
        n = write(portHandle, argv[2], strlen(argv[2]));
        if (n == -1)
        {
            printf("Error writing to serial port %s - %s(%d).\n", deviceFilePath, strerror(errno), errno);
            return EX_IOERR;
        }
        
        while (select (FD_SETSIZE, &fdset, NULL, NULL, &timeout))
        {
            // this is how you do it for blocking mode
            n = read(portHandle, &buf, sizeof(buf)-1);
            if (n == -1)
            {
                printf("Error reading from serial port %s - %s(%d).\n", deviceFilePath, strerror(errno), errno);
                return EX_IOERR;
            }
            buf[n] = '\0';
            printf("%s", buf);
        }
    }
    
    if (portHandle != -1)
    {
        close(portHandle);
        return EX_OK;
    }
    else
    {
        return EX_IOERR;
    }
}

static kern_return_t GetDevices(io_iterator_t *matchingServices)
{
    kern_return_t kernResult;
    CFMutableDictionaryRef  classesToMatch;
    
    // Serial devices are instances of class IOSerialBSDClient.
    classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);
    
    if (classesToMatch == NULL)
    {
        printf("IOServiceMatching returned a NULL dictionary.\n");
    }
    else {
        CFDictionarySetValue(classesToMatch, CFSTR(kIOSerialBSDTypeKey), CFSTR(kIOSerialBSDAllTypes));
        // Each serial device object has a property with key
        // kIOSerialBSDTypeKey and a value that is one of
        // kIOSerialBSDAllTypes, kIOSerialBSDModemType,
        // or kIOSerialBSDRS232Type. You can change the
        // matching dictionary to find other types of serial
        // devices by changing the last parameter in the above call
        // to CFDictionarySetValue.
    }
    kernResult = IOServiceGetMatchingServices(kIOMasterPortDefault, classesToMatch, matchingServices);
    
    if (KERN_SUCCESS != kernResult)
    {
        printf("IOServiceGetMatchingServices returned %d\n", kernResult);
        goto exit;
    }
    
exit:
    return kernResult;
    
}

static kern_return_t ListDevicePaths(io_iterator_t serialPortIterator, char *deviceFilePath, CFIndex maxPathSize)
{
    io_object_t device;
    kern_return_t kernResult = KERN_FAILURE;
    
    // Initialize the returned path
    *deviceFilePath = '\0';
    
    // Iterate across all devices and print them
    while ((device = IOIteratorNext(serialPortIterator)))
    {
        CFTypeRef   deviceFilePathAsCFString;
        
        // Get the path (/dev/cu.xxxxx).
        deviceFilePathAsCFString = IORegistryEntryCreateCFProperty(device, CFSTR(kIOCalloutDeviceKey), kCFAllocatorDefault, 0);
        
        if (deviceFilePathAsCFString)
        {
            Boolean result;
            // Convert the path from a CFString to a NULL-terminated C string
            result = CFStringGetCString(deviceFilePathAsCFString, deviceFilePath, maxPathSize, kCFStringEncodingASCII);
            CFRelease(deviceFilePathAsCFString);
            
            if (result)
            {
                printf("%s\n", deviceFilePath);
                kernResult = KERN_SUCCESS;
            }
        }
        
        // Release the io_service_t now that we are done with it.
        (void) IOObjectRelease(device);
    }
    
    return kernResult;
}

static int OpenSerialPort(const char *deviceFilePath, int speed)
{
    int portHandle = -1;
    struct termios options;
    
    // open the serial port, read/write, do not own it, leave it blocked (default)
    // we want blocked so we can read FULL LINES
    portHandle = open(deviceFilePath, O_RDWR | O_NOCTTY);
    
    if (portHandle == -1)
    {
        return -1;
    }
    
    // save current settings to init the options struct
    // allows us to change only items we want and leave others as they were
    if(tcgetattr(portHandle, &options) == -1)
    {
        if (portHandle != -1)
        {
            close(portHandle);
        }
        return -1;
    }
    
    // Note that open() follows POSIX semantics: multiple open() calls to
    // the same file will succeed unless the TIOCEXCL ioctl is issued.
    // This will prevent additional opens except by root-owned processes.
    // See options(4) ("man 4 options") and ioctl(2) ("man 2 ioctl") for details.
    if (ioctl(portHandle, TIOCEXCL) == -1)
    {
        if (portHandle != -1)
        {
            close(portHandle);
        }
        return -1;
    }
    
    // Set it up for 8-bit, ignore modem controls, and set the baud
    cfsetspeed(&options, speed);
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    options.c_cflag |= (CLOCAL | CREAD);// ignore modem controls
    
    // Cause the new options to take effect immediately.
    if (tcsetattr(portHandle, TCSANOW, &options) == -1)
    {
        return -1;
    }
    
    return portHandle;
}

