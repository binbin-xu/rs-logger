rs-logger
=======

Tool for logging RGB-D data from the realsense D400 series depth camera and align the depth images to colour images. Currently it was only tested using D453i sensor, but it should also work for other sensors

Currently supported output format:

- [x] Raw RGB-D sequences
- [ ] RGB-D with timestamps in TUM RGB-D format
- [x] raw: SLAMBench 1.0 file format, used also in Supereight and some other SLAM method
- [ ] Klg: used in ElasticFusion

Currently tested in Ubuntu 16.04 only. Feel free to try it out in other platforms

Supported features:

- [x] Align rgb images to depth images
- [ ] Save rgb, depth, and time stamp to local disks
- [x] Add bottom to start capturing images and press bottom to stop recording
- [x] add auto exposure
- [x] Manual tune exposure time and gain
- [x] add white balance
- [x] Read and store camera intrinsic information
- [ ] add compressed images
- [x] Add raw support 
- [ ] Add KLG support
