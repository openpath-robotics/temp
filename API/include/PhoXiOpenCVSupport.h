#pragma once
#ifndef _PHOXI_OPENCV_SUPPORT_H
#define _PHOXI_OPENCV_SUPPORT_H

#ifdef PHO_IGNORE_CV_VERSION_RESTRICTION
#    define SKIP_CV_VERSION_CHECK 1
#else
#    define SKIP_CV_VERSION_CHECK 0
#endif

#ifndef OPENCV_TRAITS_ENABLE_DEPRECATED
#    define OPENCV_TRAITS_ENABLE_DEPRECATED
#endif

#include <opencv2/core/version.hpp>
static_assert(
        SKIP_CV_VERSION_CHECK ||
        (CV_VERSION_MAJOR == 3),
        "Using unsupported OpenCV version. Only 3.x.y is supported");

#ifndef PHOXI_API_ENABLE_W4251
#    pragma warning(push)
#    pragma warning(disable:4251)
#endif

#include "PhoXiCompilerDefines.h"

#ifndef PHOXI_OPENCV_SUPPORT
#    define PHOXI_OPENCV_SUPPORT
#endif

#if (CV_VERSION_MAJOR >= 3) && (CV_VERSION_MINOR >= 3)
#    define cvDataType(X) static_cast<int>(cv::traits::Type<X>::value)
#else
#    define cvDataType(X) static_cast<int>(cv::DataType<X>::type)
#endif

#include "PhoXiDataTypes.h"

namespace pho {
namespace api {

template<class ElementType>
inline bool ConvertMat2DToOpenCVMat(
        const pho::api::Mat2D<ElementType>& Source,
        cv::Mat& Destination) {
    int DataTypeVal = -1;
    switch (ElementType::ElementChannelCount) {
    case 1:
        DataTypeVal = cvDataType(typename ElementType::ElementChannelType);
        break;
    case 2:
        DataTypeVal = cvDataType(cv::Point_<typename ElementType::ElementChannelType>);
        break;
    case 3:
        DataTypeVal = cvDataType(cv::Point3_<typename ElementType::ElementChannelType>);
        break;
    default:
        return false;
    }

    const cv::Size SourceSize(Source.Size.Width, Source.Size.Height);
    if (Destination.size() != SourceSize || Destination.type() != DataTypeVal) {
        Destination = cv::Mat(SourceSize, DataTypeVal);
    }
    const std::size_t StepSize = Source.GetStepSize(0);
    for (int row = 0; row < Destination.rows; row++) {
        memcpy(Destination.ptr<typename ElementType::ElementChannelType>(row),
                Source[row],
                StepSize * ElementType::ElementSize);
    }
    return true;
}

template<class ElementType>
inline bool ConvertOpenCVMatToMat2D(
        const cv::Mat& Source,
        pho::api::Mat2D<ElementType>& Destination) {
    if (Source.channels() != ElementType::ElementChannelCount) {
        return false;
    }
    if (ElementType::ElementChannelCount < 1 || ElementType::ElementChannelCount > 3) {
        return false;
    }

    if (Source.size() != cv::Size(Destination.Size.Width, Destination.Size.Height)) {
        Destination.Resize(PhoXiSize(Source.cols, Source.rows));
    }

    int DataTypeVal = -1;
    switch (ElementType::ElementChannelCount) {
    case 1:
        DataTypeVal = cvDataType(typename ElementType::ElementChannelType);
        break;
    case 2:
        DataTypeVal = cvDataType(typename cv::Point_<typename ElementType::ElementChannelType>);
        break;
    case 3:
        DataTypeVal = cvDataType(typename cv::Point3_<typename ElementType::ElementChannelType>);
        break;
    default:
        return false;
    }

    cv::Mat SourceConverted = Source;
    if (Source.type() != DataTypeVal) {
        SourceConverted = cv::Mat();
        Source.convertTo(SourceConverted, DataTypeVal);
    }
    const std::size_t StepSize = Destination.GetStepSize(0);
    for (int row = 0; row < SourceConverted.rows; row++) {
        memcpy(Destination[row],
                SourceConverted.ptr<const typename ElementType::ElementChannelType>(row),
                StepSize * ElementType::ElementSize);
    }
    return true;
}

#ifndef NO_DIRECT_OPENCV_SUPPORT
template<class ElementType>
bool pho::api::Mat2D<ElementType>::ConvertTo(cv::Mat& Destination) const {
    int DataTypeVal = -1;
    switch (ElementType::ElementChannelCount) {
        case 1:
            DataTypeVal = cvDataType(typename ElementType::ElementChannelType);
            break;
        case 2:
            DataTypeVal = cvDataType(cv::Point_<typename ElementType::ElementChannelType>);
            break;
        case 3:
            DataTypeVal = cvDataType(cv::Point3_<typename ElementType::ElementChannelType>);
            break;
        default:
            return false;
    }

    const cv::Size SourceSize(Size.Width, Size.Height);
    if (Destination.size() != SourceSize || Destination.type() != DataTypeVal) {
        Destination = cv::Mat(SourceSize, DataTypeVal);
    }
    for (int row = 0; row < Destination.rows; row++) {
        memcpy(Destination.ptr<typename ElementType::ElementChannelType>(row),
                this->operator[](row),
                MatInterface<ElementType, 2>::StepSize[0] * ElementType::ElementSize);
    }
    return true;
}

template<class ElementType>
bool pho::api::Mat2D<ElementType>::ConvertFrom(const cv::Mat& Source) {
    if (Source.channels() != ElementType::ElementChannelCount) {
        return false;
    }
    if (ElementType::ElementChannelCount < 1 || ElementType::ElementChannelCount > 3) {
        return false;
    }

    if (Source.size() != cv::Size(Size.Width, Size.Height)) {
        Resize(PhoXiSize(Source.cols, Source.rows));
    }

    int DataTypeVal = -1;
    switch (ElementType::ElementChannelCount) {
        case 1:
            DataTypeVal = cvDataType(typename ElementType::ElementChannelType);
            break;
        case 2:
            DataTypeVal = cvDataType(typename cv::Point_<typename ElementType::ElementChannelType>);
            break;
        case 3:
            DataTypeVal = cvDataType(typename cv::Point3_<typename ElementType::ElementChannelType>);
            break;
        default:
            return false;
    }

    cv::Mat SourceConverted = Source;
    for (int row = 0; row < SourceConverted.rows; row++) {
        memcpy(this->operator[](row),
                SourceConverted.ptr<const typename ElementType::ElementChannelType>(row),
                MatInterface<ElementType, 2>::StepSize[0] * ElementType::ElementSize);
    }
    return true;
}
#endif

} // namespace api
} // namespace pho

#ifndef PHOXI_API_ENABLE_W4251
#pragma warning(pop)
#endif

#endif //_PHOXI_OPENCV_SUPPORT_H
