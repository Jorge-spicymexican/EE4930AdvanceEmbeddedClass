#Steps to run unit test:
#These commands can just be simply done with another RunUnitTest make file
#Should attemp to see if I can include a LCOV description text for my unit test


clean:	
	rm -rf CMakeCache.txt
	rm -rf config.h 
	rm -rf CTestTestfile.cmake 
	rm -rf MakeFile
	rm -rf -d Testing
	rm -rf -d CMakeFiles
	rm -rf cmake_install.cmake
	rm -rf -d UnitTests/CMakeFiles
	rm -rf UnitTests/cmake_install.cmake
	rm -rf UnitTests/Makefile
	rm -rf UnitTests/*.exe
	rm -rf -d bsp/CMakeFiles
	rm -rf bsp/cmake_install.cmake
	rm -rf bsp/Makefile
	rm -rf bsp/main.exe
	rm -rf bsp/liblibby.a
	rm -rf -d UnitTestBuild



BuildFolder:
	mkdir UnitTestBuild
	@echo "Building the Folder: UnitTestBuild"
	@echo "!MOVE TO THIS FOLDER BEFORE CONTINUING!"
	@echo "Run command afterwards: make -f ../CustomMakefile.mk RunUnitTest"

	

RunUnitTest:
	cmake ..
	cp ../descriptions.txt descriptions.txt
	gendesc descriptions.txt -o descriptions
	@echo "Building Unit Test and running test"
	make
	make test
	@echo "============================================================="
	@echo "============================================================="
	@echo "Run Command: make -f ../CustomMakefile.mk RunLCOV"


RunLCOV:
	@echo "Running LCOV Command..." 
	lcov --capture --directory . --output-file coverage.info --test-name coverage
	genhtml coverage.info --output-directory out --title "EE4930-Advance Embedded Code-Coverage" --show-details --description-file descriptions --keep-descriptions --legend
	@echo "============================================================="
	@echo "============================================================="
	@echo "VIEW HTML : <PROJECT>/UnitTestBuild/out/index.html"
	@echo "DELETE BULD FILES: cd ../ -> make -f CustomMakefile.mk clean"
	@echo "DELETE UNITTESTBUILD AND FILES BEFORE RERUNNING LCOV!"



