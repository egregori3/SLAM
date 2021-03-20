# dependencies
pip install opencv-python  
pip install matplotlib  

# Installing lcm
https://lcm-proj.github.io/build_instructions.html, 
the reason being that it defaulted to installing for python (Python 2) but not for python3. 
setup.py has to be called from python3 install LCM for python3 as well, 
so after having installed LCM according to the instructions on https://lcm-proj.github.io/build_instructions.html
cd ../lcm-python
python3 setup.py install

Run the same thing within an activated Conda environment.


# Executing the simulation
$ python PFSimulator.py  

!ERROR! Illegal parameter  
Required: -a arenaFilename -i iterations   
Optional strings:  [-o outputPath] [-d dataFilename] [-m plot samples] [-s server IP]  
Optional ints: [-x -y -h Initial robot position] [-w Initial weight threshold] [-p numberOfParticles]  
Optional Flags: [-v verbose] [-g output graphics]  
-o must be specified with -d  
-o must be specified with -m  
-o must be specified with -g  

You need to create the test2 directory before executing the simulator.   
$ python PFSimulator.py -g -a floorplan.png -o ../../test2 -i 100 -w 0.8  


Iteration 87 sr=252,198 MT:0.779233 PT:0.014206 p=121,121 wavg=0.999294 L=251,197 TT:0.806479  
MT = Seconds to move all the particles  
PT = Seconds to run particle refactoring  
TT = Second to run the complete filter  
p = Total number of particles, number of particles not changed  
L = Predicted location of robot  
wavg = Prediction confidence  


# To generate the videos
ffmpeg -r 1 -i looking%d.jpg -vcodec libx264 -crf 25  test.mp4


# packages in environment at /home/eric/miniconda3/envs/particlefilter:
|Name|Version|Build|Channel|
|----|-------|-----|-------|
|_libgcc_mutex|0.1|main||  
|ca-certificates|2020.12.8|h06a4308_0||  
|certifi|2020.12.5|py39h06a4308_0||  
|cycler|0.10.0|pypi_0|pypi|
|kiwisolver|1.3.1|pypi_0|pypi|
|ld_impl_linux-64|2.33.1|h53a641e_7||  
|libedit|3.1.20191231|h14c3975_1||  
|libffi|3.3|he6710b0_2||  
|libgcc-ng|9.1.0|hdf63c60_0||  
|libstdcxx-ng|9.1.0|hdf63c60_0||  
|matplotlib|3.3.3|pypi_0|pypi|
|ncurses|6.2|he6710b0_1||  
|numpy|1.19.4|pypi_0|pypi|
|opencv-python|4.5.1.48|pypi_0|pypi|
|openssl|1.1.1i|h27cfd23_0||  
|pillow|8.1.0|pypi_0|pypi|
|pip|20.3.3|py39h06a4308_0||  
|pyparsing|2.4.7|pypi_0|pypi|
|python|3.9.1|hdb3f193_2||  
|python-dateutil|2.8.1|pypi_0|pypi|
|readline|8.0|h7b6447c_0||  
|setuptools|51.0.0|py39h06a4308_2||  
|six|1.15.0|pypi_0|pypi|
|sqlite|3.33.0|h62c20be_0||  
|tk|8.6.10|hbc83047_0||  
|tzdata|2020d|h14c3975_0||  
|wheel|8.36.2|pyhd3eb1b0_0||  
|xz|5.2.5|h7b6447c_0||  
|zlib|1.2.11|h7b6447c_3||  



