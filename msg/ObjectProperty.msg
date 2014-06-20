# This message defines the properties of an object


int32 id 		                         # object ID useful for manipulation

string objectType                        #type of object if defined

# x and y location of the object
float32 locationX
float32 locationY
float32 locationZ

#colour of the object
string colour

int32[3] colourHist                    # colour histogram array of the object can be modified to be 3D sift array as well