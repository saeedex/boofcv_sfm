A minimalist SfM Project based on BoofCV. 

https://github.com/lessthanoptimal/BoofCV.git

Expects a set of unordered photos with known intrinsic camera calibration.

Saves output sparse point-cloud as well as recognition models.

The build script will download all dependencies automatically.

Recommendations:
- IntelliJ for IDE
- Linux, Windows, or OSX

Useful Commands
- './gradlew assemble' to compile the project
- './gradlew boofcv_sfm' to create the boofcv_sfm jar
- 'java -jar boofcv_sfm.jar' to launch the jar. Provided so that you can test without an IDE.

