//#ifndef __IMAGE_SAVER__
//#define __IMAGE_SAVER__
//
////includes
//#include <vector>
//#include <string>
//#include <fstream>
//#include <iostream>
//using namespace std;
//
///**
// * Credit goes to Daniel Beard's Programming Blog for the base code
// * https://danielbeard.wordpress.com/2011/06/06/image-saving-code-c/
// */
//
//namespace Pathfinding {
//
////data structures
//struct Colour {
//	unsigned char r,g,b,a;
//
//	Colour() {}
//	Colour(unsigned char r, unsigned char g, unsigned char b, unsigned char a): r(r), g(g), b(b), a(a) {}
//	Colour(unsigned char r, unsigned char g, unsigned char b): r(r), g(g), b(b), a(255) {}
//};
//
//class TGAImage {
//
//private:
//	//store the pixels
//	Colour *m_pixels;
//	const short m_height;
//	const short m_width;
//
//    //Convert 2d array indexing to 1d indexing
//    inline int convert2dto1d(int x, int y) const {
//         return m_width*y + x;
//    }
//
//public:
//    TGAImage();
//    TGAImage(short width, short height);
//    ~TGAImage();
//
//    //Set all pixels at once
//    inline void setAllPixels(Colour *pixels) {
//        m_pixels = pixels;
//    }
//
//    //Set indivdual pixels
//    inline void setPixel(Colour inputcolor, int x, int y) {
//        m_pixels[convert2dto1d(x,y)] = inputcolor;
//    }
//
//    //set individual pixels, includes boundary check.
//    inline void setPixelSafe(Colour inputcolor, int x, int y) {
//        if (x < 0 || y < 0 || x >= m_width || y >= m_height) return;
//        m_pixels[convert2dto1d(x,y)] = inputcolor;
//    }
//
//    void WriteImage(string filename) const;
//};
//
//
//namespace Colours {
//    extern const Colour RED;
//    extern const Colour ORANGE;
//    extern const Colour YELLOW;
//    extern const Colour GREEN;
//    extern const Colour BLUE;
//    extern const Colour PURPLE;
//    extern const Colour MAGENTA;
//    extern const Colour BLACK;
//    extern const Colour WHITE;
//    extern const Colour BROWN;
//    extern const Colour LIGHTGREY;
//    extern const Colour GREY;
//    extern const Colour DARKGREY;
//    extern const Colour CYAN;
//    extern const Colour LIME;
//    extern const Colour AQUA;
//    extern const Colour TEAL;
//    extern const Colour PINK;
//}
//
//}
//
//#endif
