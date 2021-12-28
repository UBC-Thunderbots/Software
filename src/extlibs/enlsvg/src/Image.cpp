//#include "Image.h"
//
///**
// * Credit goes to Daniel Beard's Programming Blog for the base code
// * https://danielbeard.wordpress.com/2011/06/06/image-saving-code-c/
// */
//
//namespace Pathfinding {
//
//TGAImage::TGAImage(): m_width(0), m_height(0) {}
//
//TGAImage::TGAImage(short width, short height): m_width(width), m_height(height) {
//    m_pixels = new Colour[m_width*m_height];
//}
//
//TGAImage::~TGAImage() {
//    delete[] m_pixels;
//}
//
//void TGAImage::WriteImage(string filename) const {
//
//    //Error checking
//    if (m_width <= 0 || m_height <= 0) {
//        cout << "Image size is not set properly" << endl;
//        return;
//    }
//
//    ofstream o(filename.c_str(), ios::out | ios::binary);
//
//    //Write the header
//    o.put(0);
//    o.put(0);
//    o.put(2);                     /* uncompressed RGB */
//    o.put(0); o.put(0);
//    o.put(0); o.put(0);
//    o.put(0);
//    o.put(0); o.put(0);           /* X origin */
//    o.put(0); o.put(0);           /* y origin */
//    o.put((m_width & 0x00FF));
//    o.put((m_width & 0xFF00) / 256);
//    o.put((m_height & 0x00FF));
//    o.put((m_height & 0xFF00) / 256);
//    o.put(32);                    /* 24 bit bitmap */
//    o.put(0);
//
//    //Write the pixel data
//    for (int i=0;i<m_width*m_height;i++) {
//        o.put(m_pixels[i].b);
//        o.put(m_pixels[i].g);
//        o.put(m_pixels[i].r);
//        o.put(m_pixels[i].a);
//    }   
//
//    //close the file
//    o.close();
//}
//
//namespace Colours {
//    const Colour RED(255,0,0);
//    const Colour ORANGE(255,127,0);
//    const Colour YELLOW(255,255,0);
//    const Colour GREEN(0,255,0);
//    const Colour BLUE(0,0,255);
//    const Colour PURPLE(127,0,255);
//    const Colour MAGENTA(255,0,255);
//    const Colour BLACK(0,0,0);
//    const Colour WHITE(255,255,255);
//    const Colour BROWN(150,75,0);
//    const Colour LIGHTGREY(192,192,192);
//    const Colour GREY(127,127,127);
//    const Colour DARKGREY(64,64,64);
//    const Colour CYAN(255,255,0);
//    const Colour LIME(127,255,0);
//    const Colour AQUA(0,127,255);
//    const Colour TEAL(0,127,127);
//    const Colour PINK(255,192,203);
//}
//
//}