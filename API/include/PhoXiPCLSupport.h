#pragma once
#ifndef _PHOXI_PCL_SUPPORT_H
#define _PHOXI_PCL_SUPPORT_H

#ifndef PHOXI_API_ENABLE_W4251
#pragma warning(push)
#pragma warning(disable:4251)
#endif

#include "PhoXiCompilerDefines.h"

#ifndef PHOXI_PCL_SUPPORT
#    define PHOXI_PCL_SUPPORT
#endif

#include "PhoXiDataTypes.h"

namespace pho {
namespace api {
namespace pcls {

template<class PCLPointType>
inline bool ResizePointCloud(
        const Frame &Source,
        pcl::PointCloud<PCLPointType> &Destination) {
    PhoXiSize Resolution = Source.GetResolution();
    if (Resolution == PhoXiSize(0, 0)) {
        return false;
    }
    //Destination.resize(Resolution.Area());
    Destination = pcl::PointCloud<PCLPointType>(Resolution.Width, Resolution.Height);
    return true;
}

template<class PCLPointType>
inline bool FillPointCloudXYZ(
        const Frame &Source,
        pcl::PointCloud<PCLPointType> &Destination) {
    return false;
}

#define FillPointCloudXYZ_MACRO(elem)                            \
    template<>                                                   \
    inline bool FillPointCloudXYZ(                               \
            const Frame& Source,                                 \
            pcl::PointCloud<elem>& Destination) {                \
        if (Source.PointCloud.Empty()) {                         \
            return false;                                        \
        }                                                        \
        for (size_t i = 0; i < Destination.points.size(); ++i) { \
            memcpy(Destination.points[i].data,                   \
                    &Source.PointCloud[0][i],                    \
                    sizeof(Point3_32f));                         \
        }                                                        \
        return true;                                             \
    }

FillPointCloudXYZ_MACRO(pcl::PointXYZ);
FillPointCloudXYZ_MACRO(pcl::PointXYZI);
FillPointCloudXYZ_MACRO(pcl::PointXYZL);
FillPointCloudXYZ_MACRO(pcl::PointXYZRGBA);
FillPointCloudXYZ_MACRO(pcl::PointXYZRGB);
FillPointCloudXYZ_MACRO(pcl::PointXYZRGBL);
FillPointCloudXYZ_MACRO(pcl::PointXYZHSV);
FillPointCloudXYZ_MACRO(pcl::InterestPoint);
FillPointCloudXYZ_MACRO(pcl::PointNormal);
FillPointCloudXYZ_MACRO(pcl::PointXYZRGBNormal);
FillPointCloudXYZ_MACRO(pcl::PointXYZINormal);
FillPointCloudXYZ_MACRO(pcl::PointXYZLNormal);
FillPointCloudXYZ_MACRO(pcl::PointWithRange);
FillPointCloudXYZ_MACRO(pcl::PointWithViewpoint);
FillPointCloudXYZ_MACRO(pcl::PointWithScale);
FillPointCloudXYZ_MACRO(pcl::PointSurfel);
FillPointCloudXYZ_MACRO(pcl::PointDEM);



template<class PCLPointType>
inline bool FillPointCloudNormalXYZ(
        const Frame &Source,
        pcl::PointCloud<PCLPointType> &Destination) {
    return false;
}

#define FillPointCloudNormalXYZ_MACRO(elem)                      \
    template<>                                                   \
    inline bool FillPointCloudNormalXYZ(                         \
            const Frame& Source,                                 \
            pcl::PointCloud<elem>& Destination) {                \
        if (Source.NormalMap.Empty()) {                          \
            return false;                                        \
        }                                                        \
        for (size_t i = 0; i < Destination.points.size(); ++i) { \
            memcpy(Destination.points[i].data_n,                 \
                    &Source.NormalMap[0][i],                     \
                    sizeof(Point3_32f));                         \
        }\
        return true;\
    }

FillPointCloudNormalXYZ_MACRO(pcl::Normal);
FillPointCloudNormalXYZ_MACRO(pcl::PointNormal);
FillPointCloudNormalXYZ_MACRO(pcl::PointXYZRGBNormal);
FillPointCloudNormalXYZ_MACRO(pcl::PointXYZINormal);
FillPointCloudNormalXYZ_MACRO(pcl::PointXYZLNormal);
FillPointCloudNormalXYZ_MACRO(pcl::PointSurfel);



template<class PCLPointType>
inline bool FillPointCloudI(
        const Frame &Source,
        pcl::PointCloud<PCLPointType> &Destination) {
    return false;
}

#define FillPointCloudI_MACRO(elem)                                 \
    template<>                                                      \
    inline bool FillPointCloudI(                                    \
            const Frame& Source,                                    \
            pcl::PointCloud<elem>& Destination) {                   \
        if (Source.Texture.Empty()) {                               \
            return false;                                           \
        }                                                           \
        for (size_t i = 0; i < Destination.points.size(); ++i) {    \
            Destination.points[i].intensity = Source.Texture[0][i]; \
        }                                                           \
        return true;                                                \
    }

FillPointCloudI_MACRO(pcl::PointXYZI);
FillPointCloudI_MACRO(pcl::PointXYZINormal);



template<class PCLPointType>
inline bool FillPointCloudRGB(
        const Frame &Source,
        pcl::PointCloud<PCLPointType> &Destination,
        bool NormalizeTexture) {
    return false;
}

#define FillPointCloudRGB_MACRO(elem)                                                                                              \
    template<>                                                                                                                     \
    inline bool FillPointCloudRGB(                                                                                                 \
            const Frame& Source,                                                                                                   \
            pcl::PointCloud<elem>& Destination,                                                                                    \
            bool NormalizeTexture) {                                                                                               \
        if (Source.Texture.Empty() && Source.TextureRGB.Empty()) {                                                                 \
            return false;                                                                                                          \
        }                                                                                                                          \
        float TextureMultiplier = 1.0;                                                                                             \
        if (NormalizeTexture) {                                                                                                    \
            if (Source.Texture.Size.Area() > 0) {                                                                                  \
                const double SamplesCount = 10000.0;                                                                               \
                const double SizeRatio = (double)Source.Texture.Size.Width / (double)Source.Texture.Size.Height;                   \
                int SamplesX = (int)std::sqrt(SamplesCount * SizeRatio);                                                           \
                int SamplesY = (int)std::sqrt(SamplesCount / SizeRatio);                                                           \
                std::vector<float> Samples;                                                                                        \
                int StepY = std::max<int>(1, Source.Texture.Size.Height / SamplesY);                                               \
                int StepX = std::max<int>(1, Source.Texture.Size.Width / SamplesX);                                                \
                for (int y = 0; y < Source.Texture.Size.Height; y += StepY) {                                                      \
                    for (int x = 0; x < Source.Texture.Size.Height; x += StepY) {                                                  \
                        Samples.push_back(Source.Texture[y][x]);                                                                   \
                    }                                                                                                              \
                }                                                                                                                  \
                if (Samples.size() > 10) {                                                                                         \
                    std::sort(Samples.begin(), Samples.end());                                                                     \
                    TextureMultiplier = (255.0 / Samples[Samples.size() - 1 - Samples.size() / 98]) * 0.9;                         \
                }                                                                                                                  \
            } else if (Source.TextureRGB.Size.Area() > 0) {                                                                        \
                const double SamplesCount = 10000.0;                                                                               \
                const double SizeRatio = (double)Source.TextureRGB.Size.Width / (double)Source.TextureRGB.Size.Height;             \
                int SamplesX = (int)std::sqrt(SamplesCount * SizeRatio);                                                           \
                int SamplesY = (int)std::sqrt(SamplesCount / SizeRatio);                                                           \
                using CombinedColorT = std::pair<ColorRGB_16, float>;                                                              \
                std::vector<CombinedColorT> Samples;                                                                               \
                int StepY = std::max<int>(1, Source.TextureRGB.Size.Height / SamplesY);                                            \
                int StepX = std::max<int>(1, Source.TextureRGB.Size.Width / SamplesX);                                             \
                for (int y = 0; y < Source.TextureRGB.Size.Height; y += StepY) {                                                   \
                    for (int x = 0; x < Source.TextureRGB.Size.Height; x += StepY) {                                               \
                        const ColorRGB_16& rgb = Source.TextureRGB[y][x];                                                          \
                        Samples.emplace_back(std::make_pair(rgb, 0.11f * rgb.r + 0.59f * rgb.g + 0.3f * rgb.b));                   \
                    }                                                                                                              \
                }                                                                                                                  \
                if (Samples.size() > 10) {                                                                                         \
                    std::sort(Samples.begin(), Samples.end(),                                                                      \
                        [](const CombinedColorT& l, const CombinedColorT& r) { return l.second < r.second; });                     \
                    TextureMultiplier = (255.0 / Samples[Samples.size() - 1 - Samples.size() / 98].second) * 0.9;                  \
                }                                                                                                                  \
            }                                                                                                                      \
        }                                                                                                                          \
        if (Source.Texture.Size.Area() > 0) {                                                                                      \
            const Intensity_32f* ptr = Source.Texture.GetDataPtr();                                                                \
            for (size_t i = 0; i < Destination.points.size(); ++i) {                                                               \
                unsigned char Grey = (unsigned char)std::min<float>((ptr[i] * TextureMultiplier), 255.0f);                         \
                Destination.points[i].r = Grey;                                                                                    \
                Destination.points[i].g = Grey;                                                                                    \
                Destination.points[i].b = Grey;                                                                                    \
            }                                                                                                                      \
        } else if (Source.TextureRGB.Size.Area() > 0) {                                                                            \
            const ColorRGB_16* ptr = Source.TextureRGB.GetDataPtr();                                                               \
            for (size_t i = 0; i < Destination.points.size(); ++i) {                                                               \
                Destination.points[i].r = (unsigned char)std::min<float>((ptr[i].r * TextureMultiplier), 255.0f);                  \
                Destination.points[i].g = (unsigned char)std::min<float>((ptr[i].g * TextureMultiplier), 255.0f);                  \
                Destination.points[i].b = (unsigned char)std::min<float>((ptr[i].b * TextureMultiplier), 255.0f);                  \
            }                                                                                                                      \
        }                                                                                                                          \
        return true;                                                                                                               \
    }                                                                                                                              \

FillPointCloudRGB_MACRO(pcl::PointXYZRGBA);
FillPointCloudRGB_MACRO(pcl::PointXYZRGB);
FillPointCloudRGB_MACRO(pcl::PointXYZRGBL);
FillPointCloudRGB_MACRO(pcl::PointXYZRGBNormal);
FillPointCloudRGB_MACRO(pcl::PointSurfel);

template<class PCLPointType>
inline bool toPCLPointCloud(
        const Frame &Source,
        pcl::PointCloud<PCLPointType> &Destination,
        bool NormalizeTexture = true) {
    if (!ResizePointCloud(Source, Destination)) {
        return false;
    }
    FillPointCloudXYZ(Source, Destination);
    FillPointCloudNormalXYZ(Source, Destination);
    FillPointCloudI(Source, Destination);
    FillPointCloudRGB(Source, Destination, NormalizeTexture);
    return true;
}

} // namespace pcls


template<class PCLPointType>
inline bool Frame::ConvertTo(
        pcl::PointCloud<PCLPointType> &Destination,
        bool NormalizeTexture) const {
    return pcls::toPCLPointCloud(*this, Destination, NormalizeTexture);
}
#ifdef PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
inline bool Frame::ConvertTo(pcl::PCLPointCloud2& Destination) const {
    if (!NormalMap.Empty()){
        pcl::PointCloud<pcl::PointXYZINormal> Temporary;
        bool Result = ConvertTo(Temporary, false);
        pcl::toPCLPointCloud2(Temporary, Destination);
        return Result;
    } else {
        pcl::PointCloud<pcl::PointXYZI> Temporary;
        bool Result = ConvertTo(Temporary, false);
        pcl::toPCLPointCloud2(Temporary, Destination);
        return Result;
    }
}
#endif

} // namespace api
} // namespace pho

#ifndef PHOXI_API_ENABLE_W4251
#pragma warning(pop)
#endif

#endif //_PHOXI_PCL_SUPPORT_H