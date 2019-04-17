rs-logger
=======

Tool for logging RGB-D data from the realsense D400 series depth camera and align the depth images to colour images. Currently it was only tested using D453i sensor, but it should also work for other sensors

Currently supported output format:

- [ ] Raw RGB-D sequences, the same format as in TUM RGB-D dataset
- [ ] raw: SLAMBench 1.0 file format, used also in Supereight and some other SLAM method
- [ ] Klg: used in ElasticFusion and some other SLAM methods

Currently tested in Ubuntu 16.04 only. Feel free to try it out in other platforms

Supported features:

- [x] Align rgb images to depth images

- [ ] Save rgb, depth, and time stamp to local disks

- [x] Add bottom to start capturing images and press bottom to stop recording

- [x] add auto exposure

- [x] Manual tune exposure time and gain

- [x] add white balance

- [ ] add compress images

- [ ] Add raw support 

- [ ] Add KLG support

- [ ] Use new imgui features (ImGui::StyleColorsDark)


