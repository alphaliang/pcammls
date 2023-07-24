/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-07-18 15:55:24
 * @LastEditors: zxy
 * @LastEditTime: 2023-07-24 13:13:53
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <cmath>
#include <numeric>
#include <algorithm>
//#include <math.h>
#include <iostream>
#include <vector>
#include <turbojpeg.h>

#define MAX_DEPTH 0x10000 

static void BGRToRGB(const void* bgrFrame, int width, int height, void* rgbFrame)
{
  uint8_t* pBGR = (uint8_t*)bgrFrame;
  uint8_t* pRGB = (uint8_t*)rgbFrame;

  uint8_t r, g, b;
  for (int i = 0; i < height * width; i++)
  {
    b = *pBGR++;
    g = *pBGR++;
    r = *pBGR++;
		
    *pRGB++ = r;
    *pRGB++ = g;
    *pRGB++ = b;
  }
}


struct float3
{
    float x, y, z;

    float length() const { return sqrt(x*x + y*y + z*z); }

    float3 normalize() const
    {
        return (length() > 0)? float3{ x / length(), y / length(), z / length() }:*this;
    }
};

inline float3 cross(const float3& a, const float3& b)
{
    return { a.y * b.z - b.y * a.z, a.x * b.z - b.x * a.z, a.x * b.y - a.y * b.x };
}

inline float3 operator*(const float3& a, float t)
{
    return { a.x * t, a.y * t, a.z * t };
}

inline float3 operator/(const float3& a, float t)
{
    return { a.x / t, a.y / t, a.z / t };
}

inline float3 operator+(const float3& a, const float3& b)
{
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

inline float3 operator-(const float3& a, const float3& b)
{
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

inline float3 lerp(const float3& a, const float3& b, float t)
{
    return b * t + a * (1 - t);
}

static void update_histogram(int* hist, const uint16_t* depth_data, int w, int h)
{           
  memset(hist, 0, MAX_DEPTH * sizeof(int));
  for (auto i = 0; i < w*h; ++i) ++hist[depth_data[i]];
    for (auto i = 2; i < MAX_DEPTH; ++i) 
      hist[i] += hist[i - 1]; // Build a cumulative histogram for the     indices in [1,0xFFFF]
}       


class ty_color_map
{
public:
    ty_color_map(std::map<float, float3> map, int steps = 4000) : _map(map)
    {
        initialize(steps);
    }

    ty_color_map(const std::vector<float3>& values, int steps = 4000)
    {
        for (size_t i = 0; i < values.size(); i++)
        {
            _map[(float)i/(values.size()-1)] = values[i];
        }
        initialize(steps);
    }

    ty_color_map() {}

    float3 get(float value) const
    {
        if (_max == _min) return *_data;
        auto t = (value - _min) / (_max - _min);
        t = std::min(std::max(t, 0.f), 1.f);
        return _data[(int)(t * (_size - 1))];
    }

    float min_key() const { return _min; }
    float max_key() const { return _max; }

private:
    float3 calc(float value) const
    {
        if (_map.size() == 0) return { value, value, value };
        // if we have exactly this value in the map, just return it
        if( _map.find(value) != _map.end() ) return _map.at(value);
        // if we are beyond the limits, return the first/last element
        if( value < _map.begin()->first )   return _map.begin()->second;
        if( value > _map.rbegin()->first )  return _map.rbegin()->second;

        auto lower = _map.lower_bound(value) == _map.begin() ? _map.begin() : --(_map.lower_bound(value)) ;
            auto upper = _map.upper_bound(value);

            auto t = (value - lower->first) / (upper->first - lower->first);
            auto c1 = lower->second;
            auto c2 = upper->second;
            return lerp(c1, c2, t);
    }

    void initialize(int steps)
    {
        if (_map.size() == 0) return;

        _min = _map.begin()->first;
        _max = _map.rbegin()->first;

        _cache.resize(steps + 1);
        for (int i = 0; i <= steps; i++)
        {
            auto t = (float)i/steps;
            auto x = _min + t*(_max - _min);
            _cache[i] = calc(x);
        }

        // Save size and data to avoid STL checks penalties in DEBUG
        _size = _cache.size();
        _data = _cache.data();
    }

    std::map<float, float3> _map;
    std::vector<float3> _cache;
    float _min, _max;
    size_t _size; float3* _data;
};

static ty_color_map ty_yellow_black {{
    { 255, 255, 0 },
    { 80, 80, 0 },
}};

static ty_color_map ty_jet {{
    { 255, 0, 0 },
    { 255, 100, 0 },
    { 255, 255, 0 },
    { 0, 255, 255 },
    { 0, 0, 255 },
}};


class ImgProc 
{
public:
    enum IMGPROC_MDOE{
        IMGPROC_YUYV2RGB888,
        IMGPROC_YVYU2RGB888,
        IMGPROC_JPEG2RGB888,
        IMGPROC_BAYER8GB2RGB888,
        IMGPROC_BAYER8BG2RGB888,
        IMGPROC_BAYER8GR2RGB888,
        IMGPROC_BAYER8RG2RGB888,
        IMGPROC_BGR2RGB888,
        IMGPROC_DEPTH2RGB888,
        IMGPROC_MONO2RGB888,

        IMGPROC_CSI_MONO102RGB888,

        //G R
        //B G
        IMGPROC_CSI_BAYER10GB2RGB888,  

        //R G
        //G B
        IMGPROC_CSI_BAYER10BG2RGB888,

        //G B
        //R G
        IMGPROC_CSI_BAYER10GR2RGB888,

        //B G
        //G R
        IMGPROC_CSI_BAYER10RG2RGB888,

        IMGPROC_CSI_MONO122RGB888,
        IMGPROC_CSI_BAYER12GB2RGB888,
        IMGPROC_CSI_BAYER12BG2RGB888,
        IMGPROC_CSI_BAYER12GR2RGB888,
        IMGPROC_CSI_BAYER12RG2RGB888,

        IMGPROC_MONO162RGB888,
    };


    static void depthToRGB(const void* depthFrame, int width, int height, int* histogram, void* rgbFrame, int flag = 0) {
      unsigned short* pDepthRow = (unsigned short*)depthFrame;
      uint8_t* pRGB = (uint8_t*)rgbFrame;
      int rowSize = width;
      
      for (int y = 0; y < height; ++y)
      {   
        unsigned short* pDepth = pDepthRow;
   
        for (int x = 0; x < width; ++x, ++pDepth)
        {
          if (*pDepth != 0)
          {
            auto f = histogram[*pDepth]/(float)histogram[MAX_DEPTH - 1];
            if (!std::isnan(f))
            {
              if (flag == 0) {
                auto c = ty_yellow_black.get(f);
                *pRGB++ = static_cast<uint8_t>(c.z);
                *pRGB++ = static_cast<uint8_t>(c.y);
                *pRGB++ = static_cast<uint8_t>(c.x);
              } else if (flag == 1) {
                auto c = ty_jet.get(f);
                *pRGB++ = static_cast<uint8_t>(c.z);
                *pRGB++ = static_cast<uint8_t>(c.y);
                *pRGB++ = static_cast<uint8_t>(c.x);
              }
            }
          } else {
            pRGB += 3;
          }
        }
        pDepthRow += rowSize;
      }
    }

    static inline void monoToRGB(const void* monoFrame, int width, int height, void* rgbFrame) {
      uint8_t* pMono = (uint8_t*)monoFrame;
      uint8_t* pRGB = (uint8_t*)rgbFrame;

      uint8_t r, g, b;
      for (int i = 0; i < height * width; i++)
      {
        r = *pMono;
        g = *pMono;
        b = *pMono;
         
        *pRGB++ = r;
        *pRGB++ = g;
        *pRGB++ = b;
		
        *pMono++;
      }
    }

    static inline void parseXYZ48(int16_t* src, int16_t* dst, int width, int height, float f_scale_unit)
    {
      for (int pix = 0; pix < width*height; pix++) {
          dst[pix] =(int16_t)(*(src + 3*pix + 2) * f_scale_unit + 0.5f);
      }
    }
    static inline int decodeCsiRaw10(uint8_t* src, unsigned short* dst, int width, int height)
    {
        if(width & 0x3) {
            return -1;
        }
    
        int raw10_line_size = 5 * width / 4;
        for(size_t i = 0, j = 0; i < raw10_line_size * height; i+=5, j+=4)
        {
            //[A2 - A9] | [B2 - B9] | [C2 - C9] | [D2 - D9] | [A0A1-B0B1-C0C1-D0D1]
            dst[j + 0] = ((uint16_t)src[i + 0] << 2) | ((src[i + 4] & 0x3)  >> 0);
            dst[j + 1] = ((uint16_t)src[i + 1] << 2) | ((src[i + 4] & 0xc)  >> 2);
            dst[j + 2] = ((uint16_t)src[i + 2] << 2) | ((src[i + 4] & 0x30) >> 4);
            dst[j + 3] = ((uint16_t)src[i + 3] << 2) | ((src[i + 4] & 0xc0) >> 6);
        }
        return 0;
    }

    static inline int decodeCsiRaw12(uint8_t* src, uint16_t* dst, int width, int height)
    {
        if(width & 0x1) {
            return -1;
        }
        int raw12_line_size = 3 * width / 2;
        for(size_t i = 0, j = 0; i < raw12_line_size * height; i+=3, j+=2)
        {
            //[A4 - A11] | [B4 - B11] | [A0A1A2A3-B0B1B2B3]
            dst[j + 0] = ((uint16_t)src[i + 0] << 4) | ((src[i + 2] & 0x0f)  >> 0);
            dst[j + 1] = ((uint16_t)src[i + 1] << 4) | ((src[i + 2] & 0xf0)  >> 4);
        }
        return 0;
    }

    static inline int parseCsiRaw10(uint8_t* src, uint16_t* dst, int width, int height)
    {
        decodeCsiRaw10(src, dst, width, height);
        return 0;
    }

    static inline int parseCsiRaw12(uint8_t* src, uint16_t* dst, int width, int height)
    {
        decodeCsiRaw12(src, dst, width, height);
        return 0;
    }

    static inline void YUV2RGB(int32_t y, int32_t u, int32_t v, uint8_t* pRGB)
    {
      int32_t r, g, b;

      r = y + 1.772*(u-128);
      g = y - 0.34414*(u-128) - 0.71414*(v-128);
      b = y + 1.402*(v-128);

      if (r>255)r=255;
      if (r<0)r=0;
      if (g>255)g=255;
      if (g<0)g=0;
      if (b>255)b=255;
      if (b<0)b=0;

      *pRGB++ = b;
      *pRGB++ = g;
      *pRGB++ = r;
    }

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


#define B(x,y,w) pRGB24[0 + 3 * ((x) + w * (y))]
#define G(x,y,w) pRGB24[1 + 3 * ((x) + w * (y))]
#define R(x,y,w) pRGB24[2 + 3 * ((x) + w * (y))]
     
#define Bay(x,y,w) pBay[(x) + w * (y)]
    static inline void bayer_clear(u8 *pBay, u8 *pRGB24, int x, int y, int w)
    {
      B(x + 0, y + 0, w) = 0;
      G(x + 0, y + 0, w) = 0;
      R(x + 0, y + 0, w) = 0;

      B(x + 0, y + 1, w) = 0;
      G(x + 0, y + 1, w) = 0;
      R(x + 0, y + 1, w) = 0;

      B(x + 1, y + 0, w) = 0;
      G(x + 1, y + 0, w) = 0;
      R(x + 1, y + 0, w) = 0;

      B(x + 1, y + 1, w) = 0;
      G(x + 1, y + 1, w) = 0;
      R(x + 1, y + 1, w) = 0;
    }

    //G B
    //R G
    static inline void bayer_bilinear_gb(u8 *pBay, u8 *pRGB24, int x, int y, int w)
    {
      B(x + 0, y + 0, w) = ((u32)Bay(x - 1, y + 0, w) + (u32)Bay(x + 1, y + 0, w)) / 2;
      G(x + 0, y + 0, w) = Bay(x + 0, y + 0, w);
      R(x + 0, y + 0, w) = ((u32)Bay(x + 0, y - 1, w) + (u32)Bay(x + 0, y + 1, w)) / 2;

      B(x + 0, y + 1, w) = ((u32)Bay(x + 1, y + 0, w) + (u32)Bay(x - 1, y + 0, w) + 
                          (u32)Bay(x + 1, y + 2, w) + (u32)Bay(x - 1, y + 2, w)) / 4;
      G(x + 0, y + 1, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x + 0, y + 2, w) + 
                          (u32)Bay(x - 1, y + 1, w) + (u32)Bay(x + 1, y + 1, w)) / 4;
      R(x + 0, y + 1, w) = Bay(x + 0, y + 1, w);

      B(x + 1, y + 0, w) = Bay(x + 1, y + 0, w);
      G(x + 1, y + 0, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x + 2, y + 0, w) + 
                          (u32)Bay(x + 1, y - 1, w) + (u32)Bay(x + 1, y + 1, w)) / 4;
      R(x + 1, y + 0, w) = ((u32)Bay(x + 0, y + 1, w) + (u32)Bay(x + 2, y + 1, w) + 
                          (u32)Bay(x + 0, y - 1, w) + (u32)Bay(x + 2, y - 1, w)) / 4;

      B(x + 1, y + 1, w) = ((u32)Bay(x + 1, y + 0, w) + (u32)Bay(x + 1, y + 2, w)) / 2;
      G(x + 1, y + 1, w) = Bay(x + 1, y + 1, w);
      R(x + 1, y + 1, w) = ((u32)Bay(x + 0, y + 1, w) + (u32)Bay(x + 2, y + 1, w)) / 2;
    }

    //B G
    //G R
    static inline void bayer_bilinear_bg(u8 *pBay, u8 *pRGB24, int x, int y, int w)
    {
      B(x + 0, y + 0, w) = Bay(x + 0, y + 0, w);
      G(x + 0, y + 0, w) = ((u32)Bay(x + 0, y + 1, w) + (u32)Bay(x + 0, y - 1, w) + 
                          (u32)Bay(x - 1, y + 0, w) + (u32)Bay(x + 1, y + 0, w)) / 4;
      R(x + 0, y + 0, w) = ((u32)Bay(x - 1, y - 1, w) + (u32)Bay(x - 1, y + 1, w) + 
                          (u32)Bay(x + 1, y - 1, w) + (u32)Bay(x + 1, y + 1, w)) / 4;

      B(x + 0, y + 1, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x + 0, y + 2, w)) / 2;
      G(x + 0, y + 1, w) = Bay(x + 0, y + 1, w);
      R(x + 0, y + 1, w) = ((u32)Bay(x - 1, y + 1, w) + (u32)Bay(x + 1, y + 1, w)) / 2;
    
      B(x + 1, y + 0, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x + 2, y + 0, w)) / 2;
      G(x + 1, y + 0, w) = Bay(x + 1, y + 0, w);
      R(x + 1, y + 0, w) = ((u32)Bay(x + 1, y - 1, w) + (u32)Bay(x + 1, y + 1, w)) / 2;

      B(x + 1, y + 1, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x + 2, y + 0, w) + 
                          (u32)Bay(x + 0, y + 2, w) + (u32)Bay(x + 2, y + 2, w)) / 4;
      G(x + 1, y + 1, w) = ((u32)Bay(x + 1, y + 0, w) + (u32)Bay(x + 0, y + 1, w) + 
                          (u32)Bay(x + 1, y + 2, w) + (u32)Bay(x + 2, y + 1, w)) / 4;
      R(x + 1, y + 1, w) = Bay(x + 1, y + 1, w);
    }

    //G R
    //B G
    static inline void bayer_bilinear_gr(u8 *pBay, u8 *pRGB24, int x, int y, int w)
    {
      B(x + 0, y + 0, w) = ((u32)Bay(x + 0, y - 1, w) + (u32)Bay(x + 0, y + 1, w)) / 2;
      G(x + 0, y + 0, w) = Bay(x + 0, y + 0, w);
      R(x + 0, y + 0, w) = ((u32)Bay(x - 1, y + 0, w) + (u32)Bay(x + 1, y + 0, w)) / 2;

      B(x + 0, y + 1, w) = Bay(x + 0, y + 1, w);
      G(x + 0, y + 1, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x - 1, y + 1, w) + 
                          (u32)Bay(x + 0, y + 2, w) + (u32)Bay(x + 1, y + 1, w)) / 4;
      R(x + 0, y + 1, w) = ((u32)Bay(x - 1, y + 0, w) + (u32)Bay(x + 1, y + 0, w) + 
                          (u32)Bay(x - 1, y + 2, w) + (u32)Bay(x + 1, y + 2, w)) / 4;

      B(x + 1, y + 0, w) = ((u32)Bay(x + 0, y - 1, w) + (u32)Bay(x + 2, y - 1, w) + 
                          (u32)Bay(x + 0, y + 1, w) + (u32)Bay(x + 2, y + 1, w)) / 4;
      G(x + 1, y + 0, w) = ((u32)Bay(x + 1, y - 1, w) + (u32)Bay(x + 0, y + 0, w) + 
                          (u32)Bay(x + 2, y + 0, w) + (u32)Bay(x + 1, y + 1, w)) / 4;
      R(x + 1, y + 0, w) = Bay(x + 1, y + 0, w);
    
      B(x + 1, y + 1, w) = ((u32)Bay(x + 0, y + 1, w) + (u32)Bay(x + 2, y + 1, w)) / 2;
      G(x + 1, y + 1, w) = Bay(x + 1, y + 1, w);
      R(x + 1, y + 1, w) = ((u32)Bay(x + 1, y + 0, w) + (u32)Bay(x + 1, y + 2, w)) / 2;
    }

    //R G
    //G B
    static inline void bayer_bilinear_rg(u8 *pBay, u8 *pRGB24, int x, int y, int w)
    {
      B(x + 0, y + 0, w) = ((u32)Bay(x - 1, y - 1, w) + (u32)Bay(x + 1, y - 1, w) + 
                          (u32)Bay(x - 1, y + 1, w) + (u32)Bay(x + 1, y + 1, w)) / 4;
      G(x + 0, y + 0, w) = ((u32)Bay(x + 0, y - 1, w) + (u32)Bay(x - 1, y + 0, w) + 
                          (u32)Bay(x + 1, y + 0, w) + (u32)Bay(x + 0, y + 1, w)) / 4;
      R(x + 0, y + 0, w) = Bay(x + 0, y + 0, w);
    
      B(x + 1, y + 0, w) = ((u32)Bay(x - 1, y + 1, w) + (u32)Bay(x + 1, y + 1, w)) / 2;
      G(x + 1, y + 0, w) = Bay(x + 0, y + 1, w);
      R(x + 1, y + 0, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x + 0, y + 2, w)) / 2;

      B(x + 0, y + 1, w) = ((u32)Bay(x + 1, y - 1, w) + (u32)Bay(x + 1, y + 1, w)) / 2;
      G(x + 0, y + 1, w) = Bay(x + 1, y + 0, w);
      R(x + 0, y + 1, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x + 2, y + 0, w)) / 2;

      B(x + 1, y + 1, w) = Bay(x + 1, y + 1, w);
      G(x + 1, y + 1, w) = ((u32)Bay(x + 1, y + 0, w) + (u32)Bay(x + 0, y + 1, w) + 
                          (u32)Bay(x + 2, y + 1, w) + (u32)Bay(x + 1, y + 2, w)) / 4;
      R(x + 1, y + 1, w) = ((u32)Bay(x + 0, y + 0, w) + (u32)Bay(x + 2, y + 0, w) + 
                          (u32)Bay(x + 0, y + 2, w) + (u32)Bay(x + 2, y + 2, w)) / 4;
    }

    static int yuv2rgb(const void* src, void* dst, int width, int height, int type)
    {
      switch (type) {
        case IMGPROC_YVYU2RGB888:
        {
          const uint32_t* _src = (const uint32_t*)src;
          uint8_t*  _dst = (uint8_t*)dst;

          for (int j=0; j<height; j++) {
            const uint32_t* pYVYU = _src + (j*(width>>1));
            uint8_t* pRGB = _dst + (j*width*3);

            for (int i=0; i<(width/2); i++, pYVYU++) {
              //handle 2 pixels
              uint32_t yvyu = *pYVYU;
              uint8_t y0 = yvyu & 0x000000ff;
              uint8_t u  = (yvyu & 0x0000ff00) >> 8;
              uint8_t y1 = (yvyu & 0x00ff0000) >> 16;
              uint8_t v  = (yvyu & 0xff000000) >> 24;

              YUV2RGB(y0, u, v, pRGB); pRGB+=3;
              YUV2RGB(y1, u, v, pRGB); pRGB+=3;
            }
          }
          break;
        }
        case IMGPROC_YUYV2RGB888:
        {
          const uint32_t* _src = (const uint32_t*)src;
          uint8_t*  _dst = (uint8_t*)dst;

          for (int j=0; j<height; j++) {
            const uint32_t* pYUYV = _src + (j*(width>>1));
            uint8_t* pRGB = _dst + (j*width*3);

            for (int i=0; i<(width/2); i++, pYUYV++) {
              //handle 2 pixels
              uint32_t yuyv = *pYUYV;
              uint8_t y0 = yuyv & 0x000000ff;
              uint8_t v  = (yuyv & 0x0000ff00) >> 8;
              uint8_t y1 = (yuyv & 0x00ff0000) >> 16;
              uint8_t u  = (yuyv & 0xff000000) >> 24;

              YUV2RGB(y0, u, v, pRGB); pRGB+=3;
              YUV2RGB(y1, u, v, pRGB); pRGB+=3;
            }
          }
          break;
        }
        default:
          return -1;
      }

      return 0;
    }

    struct jpeg_decoder{
        tjhandle jpeg;
        jpeg_decoder() {jpeg = tjInitDecompress();}
        ~jpeg_decoder() {tjDestroy(jpeg);};
    };

    static int cvtColor(const image_data&src,  IMGPROC_MDOE type, image_data& dst)
    {
      int i, j;

      dst.resize(3 * src.width * src.height);
      dst.streamID     = src.streamID;
      dst.timestamp    = src.timestamp;
      dst.imageIndex   = src.imageIndex;
      dst.status       = src.status;
      dst.width        = src.width;
      dst.height       = src.height;
      dst.pixelFormat  = TY_PIXEL_FORMAT_BGR;

      switch (type) {
        case IMGPROC_YUYV2RGB888: {
          yuv2rgb(src.buffer, dst.buffer, src.width, src.height, IMGPROC_YUYV2RGB888);
          break;
        }
        case IMGPROC_YVYU2RGB888: {
          yuv2rgb(src.buffer, dst.buffer, src.width, src.height, IMGPROC_YVYU2RGB888);
          break;
        }    
        case IMGPROC_JPEG2RGB888: {
          static jpeg_decoder jpeg_de;
          
          int real_width, real_height, subsamp;
          if(-1 != tjDecompressHeader2(jpeg_de.jpeg, (uint8_t*)src.buffer, src.size, &real_width, &real_height, &subsamp))
          {
            int32_t pitch = real_width * tjPixelSize[TJPF::TJPF_RGB];
            int32_t img_size = pitch * real_height;
            if(!tjDecompress2(jpeg_de.jpeg, (uint8_t*)src.buffer, src.size, (uint8_t*)dst.buffer, real_width, pitch, real_height, TJPF::TJPF_RGB, 0))
            {
                //
            }   
          }
          break;
        }
        case IMGPROC_BAYER8GB2RGB888: {
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)src.buffer, (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_gb((u8*)src.buffer, (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_BAYER8BG2RGB888: {
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)src.buffer, (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_bg((u8*)src.buffer, (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_BAYER8GR2RGB888: {
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)src.buffer, (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_gr((u8*)src.buffer, (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_BAYER8RG2RGB888: {
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)src.buffer, (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_rg((u8*)src.buffer, (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_BGR2RGB888: {
          BGRToRGB(src.buffer, src.width, src.height, dst.buffer);
          break;
        }
        case IMGPROC_DEPTH2RGB888: {
          int colorHist[MAX_DEPTH];
          update_histogram(colorHist, (uint16_t*)src.buffer, src.width, src.height);
          depthToRGB(src.buffer, src.width, src.height, colorHist, dst.buffer, 1);
          break;
        }
        case IMGPROC_MONO2RGB888: {
          monoToRGB(src.buffer, src.width, src.height, dst.buffer);
          break;
        }
        case IMGPROC_CSI_MONO102RGB888: {
          std::vector<uint16_t> mono10(src.width * src.height);
          std::vector<uint8_t> mono8(src.width * src.height);
          parseCsiRaw10((uint8_t*)src.buffer, &mono10[0], src.width, src.height);
          for(size_t idx = 0; idx < mono10.size(); idx++) {
            mono8[idx] = mono10[idx] >> 8;
          }
          monoToRGB(&mono8[0], src.width, src.height, dst.buffer);
          break;
        }
        case IMGPROC_CSI_BAYER10GB2RGB888: {
          std::vector<uint16_t> bayer10(src.width * src.height);
          std::vector<uint8_t> bayer8(src.width * src.height);
          parseCsiRaw10((uint8_t*)src.buffer, &bayer10[0], src.width, src.height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = bayer10[idx] >> 8;
          }
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_gb((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_CSI_BAYER10BG2RGB888: {
          std::vector<uint16_t> bayer10(src.width * src.height);
          std::vector<uint8_t> bayer8(src.width * src.height);
          parseCsiRaw10((uint8_t*)src.buffer, &bayer10[0], src.width, src.height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = bayer10[idx] >> 8;
          }
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_bg((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_CSI_BAYER10GR2RGB888: {
          std::vector<uint16_t> bayer10(src.width * src.height);
          std::vector<uint8_t> bayer8(src.width * src.height);
          parseCsiRaw10((uint8_t*)src.buffer, &bayer10[0], src.width, src.height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = bayer10[idx] >> 8;
          }
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_gr((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_CSI_BAYER10RG2RGB888: {
          std::vector<uint16_t> bayer10(src.width * src.height);
          std::vector<uint8_t> bayer8(src.width * src.height);
          parseCsiRaw10((uint8_t*)src.buffer, &bayer10[0], src.width, src.height);
          for(size_t idx = 0; idx < bayer10.size(); idx++) {
            bayer8[idx] = bayer10[idx] >> 8;
          }
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_rg((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_CSI_MONO122RGB888: {
          std::vector<uint16_t> mono12(src.width * src.height);
          std::vector<uint8_t> mono8(src.width * src.height);
          parseCsiRaw12((uint8_t*)src.buffer, &mono12[0], src.width, src.height);
          for(size_t idx = 0; idx < mono12.size(); idx++) {
            mono8[idx] = mono12[idx] >> 8;
          }
          monoToRGB(&mono8[0], src.width, src.height, dst.buffer);
          break;
        }
        case IMGPROC_CSI_BAYER12GB2RGB888: {
          std::vector<uint16_t> bayer12(src.width * src.height);
          std::vector<uint8_t> bayer8(src.width * src.height);
          parseCsiRaw12((uint8_t*)src.buffer, &bayer12[0], src.width, src.height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = bayer12[idx] >> 8;
          }
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_gb((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_CSI_BAYER12BG2RGB888: {
          std::vector<uint16_t> bayer12(src.width * src.height);
          std::vector<uint8_t> bayer8(src.width * src.height);
          parseCsiRaw12((uint8_t*)src.buffer, &bayer12[0], src.width, src.height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = bayer12[idx] >> 8;
          }
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_bg((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_CSI_BAYER12GR2RGB888: {
          std::vector<uint16_t> bayer12(src.width * src.height);
          std::vector<uint8_t> bayer8(src.width * src.height);
          parseCsiRaw12((uint8_t*)src.buffer, &bayer12[0], src.width, src.height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = bayer12[idx] >> 8;
          }
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_gr((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }
        case IMGPROC_CSI_BAYER12RG2RGB888: {
          std::vector<uint16_t> bayer12(src.width * src.height);
          std::vector<uint8_t> bayer8(src.width * src.height);
          parseCsiRaw12((uint8_t*)src.buffer, &bayer12[0], src.width, src.height);
          for(size_t idx = 0; idx < bayer12.size(); idx++) {
            bayer8[idx] = bayer12[idx] >> 8;
          }
          for (i = 0; i < src.width; i += 2) {
            for (j = 0; j < src.height; j += 2) {
              if (i == 0 || j == 0 || i == src.width - 2 || j == src.height - 2)
                bayer_clear((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
              else
                bayer_bilinear_rg((u8*)&bayer8[0], (u8*)dst.buffer, i, j, src.width);
            }
          }
          break;
        }    
        case IMGPROC_MONO162RGB888: {
            uint16_t* ptr = (uint16_t*)src.buffer;
          std::vector<uint8_t> mono8(src.width * src.height);
          for(size_t idx = 0; idx < src.width * src.height; idx++) {
            mono8[idx] = ptr[idx] >> 8;
          }
          monoToRGB(&mono8[0], src.width, src.height, dst.buffer);
          break;
        }
        default:
          break;
      }

      return 0;
    }
};