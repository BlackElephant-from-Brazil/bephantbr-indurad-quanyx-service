#ifndef ImageProcessing_H
#define ImageProcessing_H

#include <IDevice.h>
#include <QObject>

/*!
 * @class ImageProcessing
 * @brief Utility class to convert frame content
 */
class ImageProcessing
{
public:

    /**
     * @brief convertBayer2RGB Convert the bayer data got from the head to its rgb representation
     * @param input the input buffer of bayer-data encoded data
     * @param output the output buffer to store the converted data
     * @param width the width, in pixel of the image
     * @param height the height, in pixel of the image
     *
     * The output buffer needs to have three times the number of pixels in the source image
     * This fonction is deprecated
     */
    static void convertBayer2RGB(const unsigned char * input, unsigned char* output, int width, int height);

    /**
     * @brief getRectifyImg Build a rectified GREYLEVEL image from a source image and the LUT
     * @param rectifiedImage the output buffer on which the image will be stored
     * @param rectifiedImageWidth the height of the rectified image
     * @param rectifiedImageHeight the width of the rectified image
     * @param lut the Look Up Table to use
     * @param rawData the source buffer of raw pixels
     * @param imageWidthPixel the number of pixels in the source image
     */
    static void getRectifyImg(unsigned char *rectifiedImage, int rectifiedImageWidth, int rectifiedImageHeight,
                       pixelCoord *lut, const unsigned char *rawData, int imageWidthPixel);

    /**
     * @brief getRectifyImgRgb Build a rectified RGB image from a source image and the LUT
     * @param rectifiedImage the output buffer on which the image will be stored
     * @param rectifiedImageWidth the height of the rectified image
     * @param rectifiedImageHeight the width of the rectified image
     * @param lut the Look Up Table to use
     * @param rgbData the source buffer of RGB pixels
     * @param imageWidthPixel the number of pixels in the source image
     */
    static void getRectifyImgRgb(unsigned char *rectifiedImage, int rectifiedImageWidth, int rectifiedImageHeight,
                          pixelCoord *lut, const unsigned char *rgbData, int imageWidthPixel);

    /**
     * @brief convertGrey162RGB Convert the Gray16 disparity data got from the head to its rgb representation by using a color LUT
     * @param input the input buffer of Gray16 data
     * @param output the output buffer to store the converted data
     * @param width the width, in pixel of the image
     * @param height the height, in pixel of the image
     */
    static void convertGray162RGB(const unsigned char * input, unsigned char* output, int width, int height);

    /**
     * @brief convertGrey82RGB Convert the Gray8 disparity data got from the head to its rgb representation by using a color LUT
     * @param input the input buffer of Gray8 data
     * @param output the output buffer to store the converted data
     * @param width the width, in pixel of the image
     * @param height the height, in pixel of the image
     */
    static void convertGray82RGB(const unsigned char * input, unsigned char* output, int width, int height);

protected:
    /**
     * @brief bayerInterpolate2x2 Interpolate color (fast)
     * @param [out] rgb three bytes computed (red, blue and green components)
     * @param input pointer to input image
     * @param x the pixel to compute (x coord)
     * @param y the pixel to compute (y coord)
     * @param imageWidth the size of input image (width)
     */
    static inline void bayerInterpolate2x2(unsigned char rgb[3], const unsigned char *input, int x, int y, int imageWidth);

    /**
     * @brief bayerInterpolate3x3 Interpolate color (slower)
     * @param [out] rgb three bytes computed (red, blue and green components)
     * @param input pointer to input image
     * @param x the pixel to compute (x coord)
     * @param y the pixel to compute (y coord)
     * @param imageWidth the size of input image (width)
     * @param imageHeight the size of input image (height)
     */
    static inline void bayerInterpolate3x3(unsigned char rgb[3], const unsigned char *input, int x, int y, int imageWidth, int imageHeight);

    /**
     * @brief bilinInterpolFastRgb
     * @param coord the array of pixel coord
     * @param rgbData the source rgb data
     * @param [out] rectifiedData the built rectified image.
     * @param imageWidthPixel the width in pixels of the image
     * @param index the index of the current pixel, in the rgbData
     */
    static inline void bilinInterpolFastRgb(const pixelCoord * coord, const unsigned char *rgbData,
                                     unsigned char* rectifiedData, int imageWidthPixel, int index);


    /**
     * @brief bilinInterpolFast
     * @param coord the pixelcoord of the current pixel
     * @param rawData the source data to use in grey level
     * @param imageWidthPixel The width of the image (in pixel)
     * @return the calculate grey level value for the current pixel
     */
    static inline unsigned char bilinInterpolFast (const pixelCoord * coord, const unsigned char *rawData, int imageWidthPixel);
};

#endif
