default:
	g++ -O2 mc_test.cpp ParabolicRamp.cpp DynamicPath.cpp Timer.cpp -lstdc++ -o Test

docs:
	doxygen doxygen.conf

simple:
	g++ -O2 test.cpp ParabolicRamp.cpp DynamicPath.cpp Timer.cpp -lstdc++ -o Test
