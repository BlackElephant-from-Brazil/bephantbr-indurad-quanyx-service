#include "omega_camera/ImageProcessing.h"



/********************************************************************************
 *  Bayer to RGB24 convertion
 ********************************************************************************/

void ImageProcessing::bayerInterpolate2x2(unsigned char rgb[3], const unsigned char *input, int x, int y, int imageWidth)
{
    struct color_s {
        int value;
        int n;
    };

    struct color_s red = { 0, 0 };
    struct color_s green = { 0, 0 };
    struct color_s blue = { 0, 0 };

    /* Interpolation using nearest neighbors (only upper-left neighbor, for performance purpose) :
     * [0][1]
     * [2][3]<- (x,y)
     * Central pixel (3) is given as parameters.
     */
    struct color_s *colors_a[4] = {
        &green, &blue,
                &red,   &green
    };
    struct color_s *colors_b[4] = {
        &green, &red,
                &blue,  &green
    };
    struct color_s *colors_c[4] = {
        &blue,  &green,
                &green, &red
    };
    struct color_s *colors_d[4] = {
        &red,   &green,
                &green, &blue
    };

    /* Check for central color */
    struct color_s **colors;
    if ((x%2==0) && (y%2==0)) {
        colors = colors_c;
    } else if ((x%2==1) && (y%2==1)) {
        colors = colors_d;
    } else if ((x%2==0) && (y%2==1)) {
        colors = colors_b;
    } else {
        colors = colors_a;
    }

    /* Check for border */
    bool borderTop = (y == 0);
    bool borderLeft = (x == 0);

    if (!borderTop) {
        if (!borderLeft) {
            colors[0]->value += *(input-imageWidth-1);
            colors[0]->n++;
        }
        colors[1]->value += *(input-imageWidth);
        colors[1]->n++;
    }
    if (!borderLeft) {
        colors[2]->value += *(input-1);
        colors[2]->n++;
    }
    colors[3]->value += *(input);
    colors[3]->n++;

    if (red.n) {
        rgb[0] = red.value / red.n;
    } else {
        rgb[0] = 0;
    }
    if (green.n) {
        rgb[1] = green.value / green.n;
    } else {
        rgb[1] = 0;
    }
    if (blue.n) {
        rgb[2] = blue.value / blue.n;
    } else {
        rgb[2] = 0;
    }
}

void ImageProcessing::bayerInterpolate3x3(unsigned char rgb[3], const unsigned char *input, int x, int y, int imageWidth, int imageHeight)
{
    struct color_s {
        int value;
        int n;
    };

    struct color_s red = { 0, 0 };
    struct color_s green = { 0, 0 };
    struct color_s blue = { 0, 0 };

    /* Interpolation using nearest neighbors :
     * [0][1][2]
     * [3][4][5]
     * [6][7][8]
     * Central pixel (4) is given as parameters.
     */
    struct color_s *colors_a[9] = {
        &green, &blue,  &green,
        &red,   &green, &red,
        &green, &blue,  &green
    };
    struct color_s *colors_b[9] = {
        &green, &red,   &green,
        &blue,  &green, &blue,
        &green, &red,   &green
    };
    struct color_s *colors_c[9] = {
        &blue,  &green, &blue,
        &green, &red,   &green,
        &blue,  &green, &blue
    };
    struct color_s *colors_d[9] = {
        &red,   &green, &red,
        &green, &blue,  &green,
        &red,   &green, &red
    };

    /* Check for central color */
    struct color_s **colors;
    if ((x%2==0) && (y%2==0)) {
        colors = colors_c;
    } else if ((x%2==1) && (y%2==1)) {
        colors = colors_d;
    } else if ((x%2==0) && (y%2==1)) {
        colors = colors_b;
    } else {
        colors = colors_a;
    }

    /* Check for border */
    bool borderTop = (y == 0);
    bool borderLeft = (x == 0);
    bool borderBottom = (y == imageHeight-1);
    bool borderRight = (x == imageWidth-1);

    if (!borderTop) {
        if (!borderLeft) {
            colors[0]->value += *(input-imageWidth-1);
            colors[0]->n++;
        }
        colors[1]->value += *(input-imageWidth);
        colors[1]->n++;
        if (!borderRight) {
            colors[2]->value += *(input-imageWidth+1);
            colors[2]->n++;
        }
    }
    if (!borderLeft) {
        colors[3]->value += *(input-1);
        colors[3]->n++;
    }
    colors[4]->value += *(input);
    colors[4]->n++;
    if (!borderRight) {
        colors[5]->value += *(input-1);
        colors[5]->n++;
    }
    if (!borderBottom) {
        if (!borderLeft) {
            colors[6]->value += *(input+imageWidth-1);
            colors[6]->n++;
        }
        colors[7]->value += *(input+imageWidth);
        colors[7]->n++;
        if (!borderRight) {
            colors[8]->value += *(input+imageWidth+1);
            colors[8]->n++;
        }
    }

    if (red.n) {
        rgb[0] = red.value / red.n;
    } else {
        rgb[0] = 0;
    }
    if (green.n) {
        rgb[1] = green.value / green.n;
    } else {
        rgb[1] = 0;
    }
    if (blue.n) {
        rgb[2] = blue.value / blue.n;
    } else {
        rgb[2] = 0;
    }
}

void ImageProcessing::convertBayer2RGB(const unsigned char * input, unsigned char* output, int width, int height)
{
    const unsigned char *data_in = input;
    unsigned char *data_out = output;
    int x, y;

    /* Bayer format is :
     * RGRGRG
     * GBGBGB
     */
    for (y = 0; y < height; ++y) {
        for (x = 0; x < width; ++x) {
            bayerInterpolate2x2(data_out, data_in, x, y, width);
            data_out += 3;
            data_in += 1;
        }
    }
}

/********************************************************************************
 *  Rectified image computation
 ********************************************************************************/

void ImageProcessing::bilinInterpolFastRgb(const pixelCoord * coord, const unsigned char *rgbData,
                                           unsigned char* rectifiedData, int imageWidthPixel, int index)
{
    unsigned int ind;
    unsigned int val,reste,n1, n2, n3, n4;
    unsigned int dx, dy, mdx;
    unsigned int v1,v2;

    // Basically the same as bilinInterpolFast, but using a multiplied by 3.
    // Iterate 3 times, because we have three colors by pixel (RGB)
    for(int iter = 0; iter < 3; iter++) {
        dx=coord->x_decimal;
        dy=coord->y_decimal;
        mdx=256-dx;

        // Get the index of this pixel
        ind = coord->ind;
        // The start index of this pixel is three times more, because each pixel is 3 bytes long
        /*
         * [R G B] [R G B] [R G B] [R G B]
         */

        ind *= 3;
        // Extract the current color of the current pixel (iter = 0 -> R, iter = 1 -> G, iter = 2 -> B)
        n1 = rgbData[ind + iter];
        ind += 3; // move to the next pixel to the right of the first one
        n2 = rgbData[ind + iter];

        v1=mdx*n1+dx*n2;

        ind += (imageWidthPixel*3); // Move to the pixel below the second one
        n4 = rgbData[ind + iter];
        ind -= 3; // Move to the pixel at the left of the second one i.e. below the first one
        n3 = rgbData[ind + iter];

        v2=mdx*n3+dx*n4;
        val=((256-dy)*v1+dy*v2);

        reste=((val&0xffff)>>15);
        // Set the calculated value for the current color in the current pixel
        rectifiedData[3*index + iter] = ((val)>>16) + reste;
    }
}

unsigned char ImageProcessing::bilinInterpolFast (const pixelCoord * coord, const unsigned char *rawData, int imageWidthPixel)
{
    unsigned int ind;
    unsigned int val,reste,n1, n2, n3, n4;
    unsigned int dx, dy, mdx;
    unsigned int v1,v2;

    dx=coord->x_decimal;
    dy=coord->y_decimal;
    mdx=256-dx;

    ind = coord->ind;
    n1 = rawData[ind];
    ind++;
    n2 = rawData[ind];

    v1=mdx*n1+dx*n2;

    ind+=imageWidthPixel;
    n4 = rawData[ind];
    ind--;
    n3 = rawData[ind];

    v2=mdx*n3+dx*n4;
    val=((256-dy)*v1+dy*v2);

    reste=((val&0xffff)>>15);
    return ((val)>>16) + reste;
}

void ImageProcessing::getRectifyImg(unsigned char *rectifiedImage, int rectifiedImageWidth, int rectifiedImageHeight,
                                    pixelCoord *lut, const unsigned char *rawData, int imageWidthPixel)
{
    int sizeCorrected = rectifiedImageWidth * rectifiedImageHeight;

    for(int i = 0; i < sizeCorrected; i++){
        rectifiedImage[i] = bilinInterpolFast(&(lut[i]), rawData, imageWidthPixel);
    }
}

void ImageProcessing::getRectifyImgRgb(unsigned char *rectifiedImage, int rectifiedImageWidth, int rectifiedImageHeight,
                                       pixelCoord *lut, const unsigned char *rgbData, int imageWidthPixel)
{
    int sizeCorrected = rectifiedImageWidth * rectifiedImageHeight;

    for(int i = 0; i < sizeCorrected; i++){
        bilinInterpolFastRgb(&(lut[i]), rgbData, rectifiedImage, imageWidthPixel, i);
    }
}
