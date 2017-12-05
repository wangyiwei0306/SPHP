This code just transplant the code of SPHP from matlab to C++

Paper "Shape-Preserving Half-Projective Warps for Image Stitching"

1 I didn't do any code optimizing so the processing time cannot meet real time demands.
2 compared to the original code there are some changes I'v made.

   1 replace the feature detecting and matching method int the original code with ORB-GMS whose source code can be found from github
     paper:(GMS: Grid-based Motion Statistics for Fast, Ultra-robust Feature Correspondence)
   2 the original code hand't give the texturemapping function's source code so I realized the texturemapping but the processing result sometimes cannot satisfy very well.
   
If you download my code please do not mind my irregular codes and if possible please do some inprovements to my codes and contact with me.

15652601883@163.com