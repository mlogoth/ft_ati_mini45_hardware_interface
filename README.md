# Linux Drivers and HW Interface for ATI Mini 45

## Drivers Installation Instructions
1. Dowload PEAK CAN-USB Linux Drivers Version 8.6.0:

   https://www.peak-system.com/Details.114+M5a4777bb44d.0.html?&L=1

2. Build Binaries
   1. ```tar –xzf peak-linux-driver-8.6.0.tar.gz```
   2. ```cd peak-linux-driver-8.6.0```
   3. ```make clean```
   4. ```make all```
   
    Be sure that the libpopt-dev package is already installed. If not:
    ```bash 
    sudo apt-get install libpopt-dev
    ```

3. Install Package:
   
   ``sudo make install``
   
   if the kernel is 5.X you get the error:
   ```cpp
   peak-linux-driver-8.6.0/driver/src/pcan_common.h:191:2: error: implicit declaration of function ‘do_gettimeofday’; did you mean ‘do_settimeofday64’? [-Werror=implicit-function-declaration]
    do_gettimeofday(tv);
    ^~~~~~~~~~~~~~~
    do_settimeofday64
    cc1: some warnings being treated as errors
    ```
    Sollution: 
    Add to peak-linux-driver-8.6.0/driver/src/pcan_common.h :
    ``` cpp
    static inline void do_gettimeofday(struct timeval *tv)
    {
	    struct timespec64 ts;
	    ktime_get_real_ts64(&ts);
	    tv->tv_sec = ts.tv_sec;
	    tv->tv_usec = ts.tv_nsec;
    }
    ```
    And run again:

    ```sh 
    make clean
    ```
    ```sh 
    make all
    ```
    ```sh
    sudo make install
    ```



4. Driver Loading:

   ``sudo modprobe pcan``


## PEAK CAN VIEW on Linux

### Install PCAN-View via repository

1.  Download and install the following file peak-system.list from the PEAK-System website:
   
   	``wget -q http://www.peak-system.com/debian/dists/`lsb_release -cs`/peak-system.list -O- | sudo tee /etc/apt/sources.list.d/peak-system.list``

2. Then, download and install the PEAK-System public key for apt-secure, so that the repository is trusted:
   
   ``wget -q http://www.peak-system.com/debian/peak-system-public-key.asc -O- | sudo apt-key add - ``

3. Install pcanview-ncurses
   
   ``sudo apt-get update && sudo apt-get install pcanview-ncurses``

### Test PCAN-View

PCAN View Connection Settings:

* Clock Frequency: 8 MHz
* Bit Rate: 1MBit/sec
  
Send Messages for Initialization:

| CAN ID | Length  |  Data | Cycle Time  | 
|:-:|:-:|:-:|:-:|
| 20Fh |  6 | 00 00 00 00 00 01  | Wait  |
| 20Fh |  6 | 20 00 00 00 00 01  | Wait  |
| 20Fh |  6 | 0D 11 0D 40 0D 41  | Wait  |
| 20Fh |  6 | 0D 12 00 00 00 01  | Wait  |
| 20Fh |  6 | 2D 11 0D 40 0D 41  | Wait  |
| 20Fh |  6 | 2D 12 00 00 00 01  | Wait  |
| 080h |  0 |   | Wait  |
