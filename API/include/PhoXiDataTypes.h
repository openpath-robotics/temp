#pragma once
#ifndef _PHOXI_DATA_TYPES_H
#define _PHOXI_DATA_TYPES_H

#ifndef PHOXI_API_ENABLE_W4251
#pragma warning(push)
#pragma warning(disable:4251)
#endif

#include "PhoXiCompilerDefines.h"

#include <string>
#include <vector>
#include <array>
#include <stdint.h>
#include <memory>
#include <functional>
#include <cstring>
#include <chrono>
#include <ctime>

#ifdef PHOXI_OPENCV_SUPPORT
#    ifndef OPENCV_TRAITS_ENABLE_DEPRECATED
#        define OPENCV_TRAITS_ENABLE_DEPRECATED
#    endif
#    include <opencv2/core/mat.hpp>
#endif

#ifdef PHOXI_PCL_SUPPORT
#    include <pcl/point_cloud.h>
#    include <pcl/point_types.h>
#endif

namespace pho {
namespace api {

typedef int32_t int32_t;
typedef uint16_t uint16_t;
typedef float float32_t;
typedef double float64_t;

/// SharedPtrCreator is a helper class for creating shared_ptr object from any instance.
/// Class detects if instance is derived from enable_shared_from_this and then create
/// shared_ptr from this.
template<typename T, class Enable = void>
struct SharedPtrCreator {
    static std::shared_ptr<T> Make(T& instance) {
        return std::make_shared<T>(instance);
    }
};

template<typename T>
struct SharedPtrCreator<T,
        typename std::enable_if<std::is_base_of<std::enable_shared_from_this<typename T::base_shared_type>, T>::value>::type> {
    static std::shared_ptr<T> Make(T& instance) {
        std::shared_ptr<T> ptr = std::static_pointer_cast<T>(instance.shared_from_this());
        return ptr;
    }
};

/// UseSharedPtrCreator using the template parameter deduction from c++11 creates the optimal
/// shared_ptr clone from the instance using helper SharedPtrCreator
template<typename T>
std::shared_ptr<T> UseSharedPtrCreator(T& instance) {
    return SharedPtrCreator<T>::Make(instance);
}

#pragma pack(push, 1)
///Simple type for storing rectangular dimensions such Resolution.
/**
The object is used to represent size of 2D elements. Width and Height
are baseic types like int/double.
*/
template<class T>
class PHOXI_DLL_API PhoXiSize_ {
public:
    T Width;
    T Height;
    PhoXiSize_();
    PhoXiSize_(T _Width, T _Height);
    PhoXiSize_(const PhoXiSize_<T> &Other);
    PhoXiSize_ &operator=(const PhoXiSize_<T> &Other);
    bool operator==(const PhoXiSize_<T> &Other) const;
    bool operator!=(const PhoXiSize_<T> &Other) const;
    T Area() const;
};
typedef PhoXiSize_<int32_t> PhoXiSize;
typedef PhoXiSize_<float64_t> PhoXiSize_64f;
#pragma pack(pop)

#pragma pack(push, 1)
///Status of the device used before connection.
class PHOXI_DLL_API PhoXiConnectionStatus {
public:
    ///Tells if the device is attached to PhoXi Control.
    bool Attached;
    ///Tells if the device is ready for connection.
    bool Ready;
    PhoXiConnectionStatus();
    PhoXiConnectionStatus(const PhoXiConnectionStatus&) = default;
    bool operator=(const PhoXiConnectionStatus &Other);
};
#pragma pack(pop)
#pragma pack(push, 1)
///Single element of channel of a Matrix of a specified Type.
/**
The object is used as a wrapper for basic types like uchar/uint/float/double.
It provides additional functionality like TypeName, or TypeSize.
*/
template<class DataType>
class MatType {
private:
    DataType Data;
public:
    static const int TypeSize = sizeof(DataType);
    static const std::string PHOXI_DLL_API TypeName;
    operator DataType &() { return Data; }
    operator const DataType &() const { return Data; }
    MatType() : Data() {}
    MatType(const MatType<DataType> &Other) : Data(Other.Data) {}
    MatType(const DataType &Other) : Data(Other) {}
    MatType<DataType> operator=(const MatType<DataType> &Other) {
        Data = Other.Data;
        return *this;
    }
    MatType<DataType> operator=(const DataType &Other) {
        Data = Other;
        return *this;
    }
    bool operator==(const MatType &Other) const {
        return (Data == Other.Data);
    }
    bool operator!=(const MatType &Other) const {
        return (Data != Other.Data);
    }
};
#pragma pack(pop)

//Forward declaration
template<class DataType>
const int MatType<DataType>::TypeSize;

#pragma pack(push, 1)
///Interface of a Matrix element object with specified Channels count.
/**
The matrix element is used to carry multi, or single channel types, like XYZ
for the PointCloud. The interface does not carry any data, just the information
about the type.
*/
template<class DataType, int ChannelCount>
class MatElementInterface {
public:
    static const int ElementChannelCount = ChannelCount;
    static const int ElementSize = MatType<DataType>::TypeSize * ChannelCount;
    static const int ElementChannelSize = MatType<DataType>::TypeSize;
    typedef DataType ElementChannelType;
    static std::string GetElementName() {
        return MatType<DataType>::TypeName + "C" + std::to_string(ChannelCount);
    }
};
#pragma pack(pop)

template<class DataType, int ChannelCount>
const int MatElementInterface<DataType, ChannelCount>::ElementChannelCount;
template<class DataType, int ChannelCount>
const int MatElementInterface<DataType, ChannelCount>::ElementSize;
template<class DataType, int ChannelCount>
const int MatElementInterface<DataType, ChannelCount>::ElementChannelSize;

#pragma pack(push, 1)
///Three channel Matrix element.
/**
3 Dimensional Point or Vector of a specified ChannelType.
*/
template<class ChannelType>
class Point3 : public MatElementInterface<ChannelType, 3> {
public:
    typedef ChannelType ElementChannelType;
    MatType<ChannelType> x;
    MatType<ChannelType> y;
    MatType<ChannelType> z;
    Point3() : x(ChannelType()), y(ChannelType()), z(ChannelType()) {}
    Point3(const ChannelType &_x, const ChannelType &_y, const ChannelType &_z) : x(_x), y(_y), z(_z) {}
    bool operator==(const Point3 &Other) const {
        return (x == Other.x && y == Other.y && z == Other.z);
    }
    bool operator!=(const Point3 &Other) const {
        return (x != Other.x || y != Other.y || z != Other.z);
    }
};
typedef Point3<float32_t> Point3_32f;
typedef Point3<float64_t> Point3_64f;
#pragma pack(pop)

#pragma pack(push, 1)
///Two channel Matrix element.
/**
2 Dimensional Point or Vector of a specified ChannelType.
*/
template<class ChannelType>
class Point2 : public MatElementInterface<ChannelType, 2> {
public:
    typedef ChannelType ElementChannelType;
    MatType<ChannelType> x;
    MatType<ChannelType> y;
    Point2() : x(ChannelType()), y(ChannelType()) {}
    Point2(const ChannelType &_x, const ChannelType &_y) : x(_x), y(_y) {}
    bool operator==(const Point2 &Other) const {
        return (x == Other.x && y == Other.y);
    }
    bool operator!=(const Point2 &Other) const {
        return (x != Other.x || y != Other.y);
    }
};
typedef Point2<float32_t> Point2_32f;
#pragma pack(pop)

#pragma pack(push, 1)
///single channel Matrix element.
template<class ChannelType>
class Scalar : public MatElementInterface<ChannelType, 1>, public MatType<ChannelType> {
public:
    typedef ChannelType ElementChannelType;
    Scalar() : MatType<ChannelType>() {}
    Scalar(const Scalar &Other) : MatType<ChannelType>(Other) {}
    Scalar(const ChannelType &Other) : MatType<ChannelType>(Other) {}
    Scalar& operator=(const Scalar&) = default;
};

typedef Scalar<uint8_t> Intensity_8;
typedef Scalar<uint16_t> Intensity_16;
typedef Scalar<float32_t> Intensity_32f;
typedef Scalar<float32_t> Depth_32f;
typedef Scalar<float64_t> Depth_64f;
#pragma pack(pop)

#pragma pack(push, 1)
///Simple type for storing rectangular dimensions such Resolution.
template<class ChannelType>
class AxisVolume {
public:
    Point3<ChannelType> min, max;
    AxisVolume() {
        min = Point3<ChannelType>(0.0, 0.0, 0.0);
        max = Point3<ChannelType>(0.0, 0.0, 0.0);
    }
    AxisVolume(const Point3<ChannelType>& _min, const Point3<ChannelType>& _max) {
        min = _min;
        max = _max;
    }
    AxisVolume(const AxisVolume &Other) {
        min = Other.min;
        max = Other.max;
    }
    AxisVolume &operator=(const AxisVolume &Other) {
        min = Other.min;
        max = Other.max;
        return *this;
    }
    bool operator==(const AxisVolume &Other) const {
        return min == Other.min && max == Other.max;
    }
};
typedef AxisVolume<float32_t> AxisVolume_32f;
typedef AxisVolume<float64_t> AxisVolume_64f;
#pragma pack(pop)

#pragma pack(push, 1)
template<class ChannelType>
class Plane {
public:
    Point3<ChannelType> normal;
    ChannelType d;
    Plane() : d(0.0) {}
    Plane(const Plane& Other) = default;
    Plane(const Point3<ChannelType>& _normal, const ChannelType& _d) : normal(_normal), d(_d) {}
    Plane& operator=(const Plane& Other) = default;
    bool operator==(const Plane<ChannelType>& Other) const {
        return (normal == Other.normal && d == Other.d);
    };
    bool operator!=(const Plane<ChannelType>& Other) const {
        return !operator==(Other);
    };
};
typedef Plane<float64_t> Plane_64f;
#pragma pack(pop)

#pragma pack(push, 1)
///3 channel matrix element for storing information about bgr color
template<class ChannelType>
class ColorBGR : public MatElementInterface<ChannelType, 3> {
public:
    MatType<ChannelType> b;
    MatType<ChannelType> g;
    MatType<ChannelType> r;
    ColorBGR() : b(ChannelType()), g(ChannelType()), r(ChannelType()) {}
    ColorBGR(const ChannelType &_b, const ChannelType &_g, const ChannelType &_r) : b(_b), g(_g), r(_r) {}
    bool operator==(const ColorBGR &Other) const {
        return (b == Other.b && g == Other.g && r == Other.r);
    }
    bool operator!=(const ColorBGR &Other) const {
        return (b != Other.b || g != Other.g || r != Other.r);
    }
};

typedef ColorBGR<uint8_t> ColorBGR_8;
typedef ColorBGR<float32_t> ColorBGR_32f;

#pragma pack(pop)

#pragma pack(push, 1)
///3 channel matrix element for storing information about bgr color.
template<class ChannelType>
class ColorRGB : public MatElementInterface<ChannelType, 3> {
public:
    MatType<ChannelType> r;
    MatType<ChannelType> g;
    MatType<ChannelType> b;
    ColorRGB() : r(ChannelType()), g(ChannelType()), b(ChannelType()) {}
    ColorRGB(const ChannelType &_r, const ChannelType &_g, const ChannelType &_b) : r(_r), g(_g), b(_b) {}
    bool operator==(const ColorRGB &Other) const {
        return (b == Other.b && g == Other.g && r == Other.r);
    }
    bool operator!=(const ColorRGB &Other) const {
        return (b != Other.b || g != Other.g || r != Other.r);
    }
};

typedef ColorRGB<uint8_t> ColorRGB_8;
typedef ColorRGB<uint16_t> ColorRGB_16;

#pragma pack(pop)

template<class T>
struct MatInterfaceNoDelete {
    void operator()(T *ptr) const {
    }
};

template<class T>
void MatInterfaceNoDeleteAction(T Ptr[]) {
    return;
}

#pragma pack(push, 1)
///Interface of a Matrix object.
/**
Matrices are used for storing Frame outputs such as PointClouds
or Textures. General n-dimensional Matrix interface.
*/
template<class ElementType, int Dimension>
class MatInterface : public std::enable_shared_from_this<MatInterface<ElementType, Dimension>> {
public:
    typedef MatInterface<ElementType, Dimension> base_shared_type;

    enum OwnershipTransfer {
        CopyData,
        NoCopyNoOwnership,
        NoCopyFullOwnership
    };
protected:
    std::unique_ptr <ElementType, std::function<void(ElementType *)>> Data;
    std::size_t ElementsCount;
    std::size_t DimensionSize[Dimension];
    std::size_t StepSize[Dimension];
    void SetSizeND(const std::vector<int> &SizeND) {
        StepSize[Dimension - 1] = 1;
        for (std::size_t i = 0; i < SizeND.size(); i++) {
            DimensionSize[i] = SizeND[i];
        }
        for (int i = Dimension - 2; i >= 0; i--) {
            StepSize[i] = StepSize[i + 1] * DimensionSize[i + 1];
        }
    }
    std::size_t GetElementsCountInternal() const {
        std::size_t Result = DimensionSize[0];
        for (std::size_t i = 1; i < Dimension; i++) {
            Result *= DimensionSize[i];
        }
        return Result;
    }
    std::size_t GetElementsCountInternal(const std::vector<int> &SizeND) const {
        std::size_t Result = SizeND[0];
        for (std::size_t i = 1; i < SizeND.size(); i++) {
            Result *= SizeND[i];
        }
        return Result;
    }
    /**
    Reshape the Matrix to be n-dimensional SizeND[0], SizeND[1], ... SizeND[n-1].
    The Data will not be touched - this is O(1) operation.
    */
    bool ReshapeInternal(const std::vector<int> &SizeND) {
        if ((int) SizeND.size() != Dimension) return false;
        std::size_t ElementsCountNew = GetElementsCountInternal(SizeND);
        if (ElementsCountNew != ElementsCount) return false;
        SetSizeND(SizeND);
        return true;
    }
    /**
    Reshape the Matrix to be n-dimensional SizeND[0], SizeND[1], ... SizeND[n-1]
    and reallocate data if needed.
    */
    bool ResizeInternal(const std::vector<int> &SizeND) {
        if ((int) SizeND.size() != Dimension) return false;
        std::size_t ElementsCountNew = GetElementsCountInternal(SizeND);

        if (ElementsCountNew != ElementsCount) {
            Data = std::unique_ptr < ElementType, std::function< void(ElementType*) >> (new ElementType[ElementsCountNew], [](ElementType* Ptr) { delete[] Ptr; });
        ElementsCount = ElementsCountNew;
        }
        return ReshapeInternal(SizeND);
    }
    bool AssignInternal(const std::vector<int> &SizeND, ElementType *Data_, OwnershipTransfer DataOwnershipTransfer) {
        switch (DataOwnershipTransfer) {
            case OwnershipTransfer::CopyData:
                if (!Resize(SizeND)) return false;
                std::memcpy(GetDataPtr(), Data_, GetDataSize());
                break;
            case OwnershipTransfer::NoCopyNoOwnership:
                SetSizeND(SizeND);
                Data = std::unique_ptr < ElementType, std::function
                    < void(ElementType * ) >> (Data_, [](ElementType *Ptr) {});
                ElementsCount = GetElementsCountInternal();
                break;
            case OwnershipTransfer::NoCopyFullOwnership:
                SetSizeND(SizeND);
                Data = std::unique_ptr < ElementType, std::function
                    < void(ElementType * ) >> (Data_, [](ElementType *Ptr) { delete[] Ptr; });
                ElementsCount = GetElementsCountInternal();
                break;
        }
        return ReshapeInternal(SizeND);
    }
public:
    virtual ~MatInterface() = default;
    ///Get a list of all dimension sizes.
    std::vector<int> GetDimensions() const {
        std::vector<int> Result((std::size_t) Dimension);
        for (int i = 0; i < Dimension; i++) {
            Result[i] = (int) DimensionSize[i];
        }
        return Result;
    }
    ///Get i-th dimension size.
    int GetDimension(int Index) const {
        return (int) DimensionSize[Index];
    }
    ///Get the number of Matrix elements.
    std::size_t GetElementsCount() const {
        return ElementsCount;
    }
    ///Get Data Size of whole Matrix in Bytes.
    std::size_t GetDataSize() const {
        return GetElementsCount() * sizeof(ElementType);
    }
    ///Get the size of the Matrix Element in bytes.
    static std::size_t GetElementSize() {
        return sizeof(ElementType);
    }
    ///Get the Name of the Matrix Element.
    static std::string GetElementName() {
        return ElementType::GetElementName();
    }
    virtual bool Reshape(const std::vector<int> &SizeND) = 0;
    virtual bool Resize(const std::vector<int> &SizeND) = 0;
    virtual bool Assign(const std::vector<int> &SizeND,
                        ElementType *Data_,
                        OwnershipTransfer DataOwnershipTransfer) = 0;
    virtual ElementType &At(const std::vector<int> &Index) {
        std::size_t ArrayIndex = 0;
        for (std::size_t i = 0; i < Index.size(); i++) {
            ArrayIndex += Index[i] * StepSize[i];
        }
        return Data.get()[ArrayIndex];
    }
    virtual const ElementType &At(const std::vector<int> &Index) const {
        std::size_t ArrayIndex = 0;
        for (std::size_t i = 0; i < Index.size(); i++) {
            ArrayIndex += Index[i] * StepSize[i];
        }
        return Data.get()[ArrayIndex];
    }
    MatInterface(const MatInterface<ElementType, Dimension> &Other)
        : std::enable_shared_from_this<base_shared_type>(Other), ElementsCount(0) {
        if (ResizeInternal(Other.GetDimensions())) {
            std::memcpy(GetDataPtr(), Other.GetDataPtr(), GetDataSize());
        }
    }
    MatInterface(const std::vector<int> &SizeND)
        : std::enable_shared_from_this<base_shared_type>(), ElementsCount(0) {
        ResizeInternal(SizeND);
    }
    MatInterface(const std::vector<int> &SizeND, ElementType *Data, OwnershipTransfer DataOwnershipTransfer)
        : std::enable_shared_from_this<base_shared_type>(), ElementsCount(0) {
        AssignInternal(SizeND, Data, DataOwnershipTransfer);
    }
    MatInterface()
        : std::enable_shared_from_this<base_shared_type>(), ElementsCount(0) {
        for (int i = 0; i < Dimension; i++) {
            DimensionSize[i] = 0;
            StepSize[i] = 0;
        }
        StepSize[Dimension - 1] = 1;
    }
    bool operator=(const MatInterface<ElementType, Dimension> &Other) {
        if (Resize(Other.GetDimensions())) {
            std::memcpy(GetDataPtr(), Other.GetDataPtr(), GetDataSize());
            return true;
        }
        return false;
    }
    bool operator=(const ElementType &Element) {
        std::fill(Data[0], Data[ElementsCount - 1], Element);
        return true;
    }
    ///Get pointer to Raw data.
    ElementType *GetDataPtr() {
        if (!Data) return NULL;
        return Data.get();
    }
    ///Get const pointer to Raw data.
    const ElementType *GetDataPtr() const {
        if (!Data) return NULL;
        return Data.get();
    }
    ///Returns true if the Matrix does not contain data.
    bool Empty() const {
        return (ElementsCount == 0);
    }
    ///Clears the Matrix data and size.
    bool Clear() {
        for (int i = 0; i < Dimension; i++) {
            DimensionSize[i] = 0;
            StepSize[i] = 0;
        }
        StepSize[Dimension - 1] = 1;
        Data = nullptr;
        ElementsCount = 0;
        return true;
    }
    std::size_t GetStepSize(int Dimenstion) const {
        return StepSize[Dimenstion];
    }
};

#pragma pack(pop)

#pragma pack(push, 1)
///2D Matrix.
/**
Matrices are used for storing Frame outputs such as PointClouds or Textures.
Most common matrix type, widely used for organized Frame like structures.
*/
template<class ElementType>
class Mat2D : public MatInterface<ElementType, 2> {
private:
    void UpdateSizeInternal() {
        Size.Height = (int) MatInterface<ElementType, 2>::DimensionSize[0];
        Size.Width = (int) MatInterface<ElementType, 2>::DimensionSize[1];
    }
    std::vector<int> PhoXiSize2SizeND(const PhoXiSize &Size) const {
        std::vector<int> Result(2);
        Result[0] = Size.Height;
        Result[1] = Size.Width;
        return Result;
    }
public:
    ///Read-only size information.
    PhoXiSize Size;
    /**
    Reshape the Matrix to be n-dimensional SizeND[0], SizeND[1], ... SizeND[n-1].
    The Data will not be touched - this is O(1) operation.
    */
    virtual bool Reshape(const std::vector<int> &SizeND) override {
        bool Result = MatInterface<ElementType, 2>::ReshapeInternal(SizeND);
        if (Result) {
            UpdateSizeInternal();
        }
        return Result;
    }
    /**
    Reshape the matrix to be 2 dimensional Size.Height x Size.Width.
    The Data will not be touched - this is O(1) operation.
    */
    virtual bool Reshape(const PhoXiSize &Size) {
        bool Result = MatInterface<ElementType, 2>::ReshapeInternal(PhoXiSize2SizeND(Size));
        if (Result) {
            UpdateSizeInternal();
        }
        return Result;
    }
    /**
    Reshape the Matrix to be n-dimensional SizeND[0], SizeND[1], ... SizeND[n-1]
    and reallocate data if needed.
    */
    virtual bool Resize(const std::vector<int> &SizeND) override {
        bool Result = MatInterface<ElementType, 2>::ResizeInternal(SizeND);
        if (Result) {
            UpdateSizeInternal();
        }
        return Result;
    }
    /**
    Reshape the matrix to be 2 dimensional Size.Height x Size.Width
    and reallocate data if needed.
    */
    virtual bool Resize(const PhoXiSize &Size) {
        bool Result = MatInterface<ElementType, 2>::ResizeInternal(PhoXiSize2SizeND(Size));
        if (Result) {
            UpdateSizeInternal();
        }
        return Result;
    }
    ///Access to a specified matrix element of 2D matrix.
    virtual ElementType &At(int y, int x) {
        return MatInterface<ElementType, 2>::Data.get()[y * MatInterface<ElementType, 2>::StepSize[0] + x];
    }
    ///Access to a specified matrix element of 2D matrix by const reference.
    virtual const ElementType &At(int y, int x) const {
        return MatInterface<ElementType, 2>::Data.get()[y * MatInterface<ElementType, 2>::StepSize[0] + x];
    }
    ///Access to a specified matrix row of 2D matrix.
    ElementType *operator[](int y) {
        return &MatInterface<ElementType, 2>::Data.get()[y * MatInterface<ElementType, 2>::StepSize[0]];
    }
    ///Access to a specified matrix row of 2D matrix by const reference.
    const ElementType *operator[](int y) const {
        return &MatInterface<ElementType, 2>::Data.get()[y * MatInterface<ElementType, 2>::StepSize[0]];
    }
    ///Get the data copy of specified channel alone.
    bool GetChannelCopy(Mat2D<Scalar<typename ElementType::ElementChannelType>> &OutputMat, int ChannelIndex) const {
        if (ChannelIndex >= ElementType::ElementChannelCount) {
            return false;
        }
        if (!OutputMat.Resize(Size)) return false;
        Scalar<typename ElementType::ElementChannelType> *OutputPixel;
        const Scalar<typename ElementType::ElementChannelType> *InputPixel;
        for (decltype(Size.Height) y = 0; y < Size.Height; y++) {
            InputPixel = (const Scalar<typename ElementType::ElementChannelType> *) &At(y, 0);
            InputPixel += ChannelIndex;
            OutputPixel = &OutputMat.At(y, 0);
            for (decltype(Size.Width) x = 0; x < Size.Width; x++) {
                *OutputPixel = *InputPixel;
                ++OutputPixel;
                InputPixel += ElementType::ElementChannelCount;
            }
        }
        return true;
    }
    Mat2D(const Mat2D &Other) : MatInterface<ElementType, 2>(Other) { UpdateSizeInternal(); }
    Mat2D(const std::vector<int> &SizeND) : MatInterface<ElementType, 2>(SizeND) { UpdateSizeInternal(); }
    Mat2D(const std::vector<int> &SizeND,
          ElementType *_Data,
          typename MatInterface<ElementType, 2>::OwnershipTransfer DataOwnershipTransfer) : MatInterface<ElementType,
                                                                                                         2>(SizeND,
                                                                                                            _Data,
                                                                                                            DataOwnershipTransfer) { UpdateSizeInternal(); }
    Mat2D(const PhoXiSize &Size) : MatInterface<ElementType, 2>(PhoXiSize2SizeND(Size)) { UpdateSizeInternal(); }
    Mat2D(const PhoXiSize &Size,
          ElementType *_Data,
          typename MatInterface<ElementType, 2>::OwnershipTransfer DataOwnershipTransfer) : MatInterface<ElementType,
                                                                                                         2>(
        PhoXiSize2SizeND(Size),
        _Data,
        DataOwnershipTransfer) { UpdateSizeInternal(); }
    Mat2D() : MatInterface<ElementType, 2>() { UpdateSizeInternal(); }
    Mat2D& operator=(const Mat2D&) = default;

    ///Assign data from an external source.
    /**
    You can choose of multiple types of DataOwnershipTransfer.
    CopyData - Data will be copied and freed in the object destructor.
    NoCopyNoOwnership - Data will reference to remote pointer and the data will
        not be freed in the object destructor.
    NoCopyFullOwnership - Data will reference to remote pointer and the data
        will be copied in the object destructor.
    */
    virtual bool Assign(const PhoXiSize &Size,
                        ElementType *_Data,
                        typename MatInterface<ElementType, 2>::OwnershipTransfer DataOwnershipTransfer) {
        const bool
            Result = MatInterface<ElementType, 2>::AssignInternal(PhoXiSize2SizeND(Size), _Data, DataOwnershipTransfer);
        UpdateSizeInternal();
        return Result;
    }
    ///Assign data from an external source - general case.
    /**
    You can choose of multiple types of DataOwnershipTransfer.
    CopyData - Data will be copied and freed in the object destructor.
    NoCopyNoOwnership - Data will reference to remote pointer and the data
        will not be freed in the object destructor.
    NoCopyFullOwnership - Data will reference to remote pointer and the data
    will be copied in the object destructor.
    */
    virtual bool Assign(const std::vector<int> &SizeND,
                        ElementType *_Data,
                        typename MatInterface<ElementType, 2>::OwnershipTransfer DataOwnershipTransfer) {
        return MatInterface<ElementType, 2>::AssignInternal(SizeND, _Data, DataOwnershipTransfer);
    }

#ifdef PHOXI_OPENCV_SUPPORT
    /**
    * Convert `Mat2D` to OpenCV `cv::Mat` object - using copy.
    *
    * @param Destination [in,out] Output parameter, will be reallocated
    * @return `true` on Success, `false` otherwise.
    */
    bool ConvertTo(cv::Mat& Destination) const;

    /**
    * Convert OpenCV `cv::Mat` object to `Mat2D` - using copy.
    *
    * @param Source [in] This object will be reallocated if necessary, if the
    * `Source` have incorrect number of channels, the call will fail and return
    * false, otherwise it will be converted, if needed.
    * @return `true` on Success, `false` otherwise.
    */
    bool ConvertFrom(const cv::Mat& Source);
#else
#   define NO_DIRECT_OPENCV_SUPPORT
#endif

    void ConvertTo2DArray(float *mat2DArray, int dim1, int dim2);
    void ConvertTo3DArray(float *mat2DArray, int dim1, int dim2, int dim3);

};
#pragma pack(pop)

template<class ElementType>
void Mat2D<ElementType>::ConvertTo3DArray(float *mat2DArray, int dim1, int dim2, int dim3) {
    for (int y = 0; y < dim1; y++) {
        memcpy(mat2DArray,
               this->operator[](y),
               MatInterface<ElementType, 2>::StepSize[0] * ElementType::ElementSize);
        mat2DArray += dim2*dim3;
    }
}

template<class ElementType>
void Mat2D<ElementType>::ConvertTo2DArray(float *mat2DArray, int dim1, int dim2) {
    for (int y = 0; y < dim1; y++) {
        memcpy(mat2DArray,
               this->operator[](y),
               MatInterface<ElementType, 2>::StepSize[0] * ElementType::ElementSize);
        mat2DArray += dim2;
    }
}

typedef Mat2D<Point3_32f> PointCloud32f;
typedef Mat2D<Point3_32f> NormalMap32f;
typedef Mat2D<Point3_32f> ReprojectionMap32f;
typedef Mat2D<Depth_32f> DepthMap32f;
typedef Mat2D<Intensity_32f> ConfidenceMap32f;
typedef Mat2D<Intensity_32f> Texture32f;
typedef Mat2D<ColorRGB_16> TextureRGB16;
typedef Mat2D<Depth_64f> RotationMatrix64f;
typedef Mat2D<Depth_64f> CameraMatrix64f;
typedef Mat2D<Intensity_32f> EventMap32f;

typedef std::shared_ptr <PointCloud32f> PPointCloud32f;
typedef std::shared_ptr <PointCloud32f> PNormalMap32f;
typedef std::shared_ptr <DepthMap32f> PDepthMap32f;
typedef std::shared_ptr <ConfidenceMap32f> PConfidenceMap32f;
typedef std::shared_ptr <Texture32f> PTexture32f;
typedef std::shared_ptr <TextureRGB16> PTextureRGB16;
typedef std::shared_ptr <EventMap32f> PIndexEventMap32f;

#pragma pack(push, 1)
class PHOXI_DLL_API PhoXiMesh {
public:
    // Scanning volume cross section points, flattened in one vector in the coordinate system of the primary camera.
    std::vector<Point3_64f> Vertices;
    
    // Count of vertex points per cross section.
    unsigned int PointsPerSection;

    // The indices into the Vertices vector,
    // based on which the triangles are obtained, which then form the mesh.
    // Every three indices (counting from [0]) represents one triangle.
    std::vector<unsigned int> Indices;

    // Count of cross sections used to represent the scanning volume boundary mesh.
    unsigned int CrossSectionCount() const { return IsValid() ? static_cast<unsigned int>(Vertices.size() / PointsPerSection) : 0; }

    // Return true if mesh is valid.
    bool IsValid() const { return PointsPerSection >= 3 && Vertices.size() >= 3 && !Indices.empty() && (Indices.size() % 3 == 0); }

    PhoXiMesh() : PointsPerSection(0) {};
    bool operator==(const PhoXiMesh &Other) const;
    bool operator!=(const PhoXiMesh &Other) const;
};
#pragma pack(pop)

#pragma pack(push, 1)
class PHOXI_DLL_API ProjectionGeometry_64f {
public:
    // Origin of the projector given in the coordinate system of the primary camera.
    Point3<float64_t> Origin;
    // Tangential vectors of the projection edges given in the coordinate system of the primary camera.
    Point3<float64_t> TopLeftTangentialVector;
    Point3<float64_t> TopRightTangentialVector;
    Point3<float64_t> BottomLeftTangentialVector;
    Point3<float64_t> BottomRightTangentialVector;
    // Projection top and bottom contours are conic surfaces. We provide
    // tangential vectors of lines contained in those cones, ordered from
    // left to right and sampled to 50 points, including the above edge vectors.
    // Note: count is controlled via env. variable (PHOXI_CONTOUR_POINTS_COUNT).
    std::vector<Point3<float64_t>> TopContourPoints;
    std::vector<Point3<float64_t>> BottomContourPoints;

    ProjectionGeometry_64f() = default;
    ProjectionGeometry_64f(const ProjectionGeometry_64f& Other) = default;
    ProjectionGeometry_64f& operator=(const ProjectionGeometry_64f& Other) = default;
    bool operator==(const ProjectionGeometry_64f& Other) const;
    bool operator!=(const ProjectionGeometry_64f& Other) const;
};
#pragma pack(pop)

//Forward Declaration for internal purposes
class PhoXiCoordinateTransformationInternal;
#pragma pack(push, 8)
///Defines transformation of the coordinate space.
class PHOXI_DLL_API PhoXiCoordinateTransformation {
private:
    friend class PhoXiCoordinateTransformationInternal;
    ///Defines if the coordinate transformation is supported.
    bool Supported;
public:
    ///3 x 3 Rotation Matrix.
    RotationMatrix64f Rotation;
    ///3D Translation vector.
    Point3_64f Translation;
    PhoXiCoordinateTransformation();
    PhoXiCoordinateTransformation(const PhoXiCoordinateTransformation&) = default;
    bool isSupported() const;
    bool operator=(const PhoXiCoordinateTransformation &Other);
    bool operator==(const PhoXiCoordinateTransformation &Other) const;
};
#pragma pack(pop)

enum TemperatureDeviceSensor {
    ProjectionUnitBoard = 0,
    ProjectionUnitLaser,
    ProjectionUnitGalvo,
    CameraSensorBoard,
    CameraRearHousing,
    CameraFrontHousing,
    CameraSensorDie,
    CameraSensorDieRaw,
    CameraInterfaceBoard,
    TempsCount
};

//Temperature not available.
static const double PHOXI_DEVICE_TEMPERATURE_INVALID = -273.15;

struct PhoXiMarkerDots {
    // NOTE: this enum has its counterpart pho::MarkerDotCorrector::Status
    //       within PhoXi in MarkerDorCorrector.h
    //       and it **must** be kept in sync.
    //       The only acceptable change is thus addition of new values.
    enum MarkerDotStatus {
        ///Uninitialized value, should not come to end user
        UNKNOWN = 0,
        ///The feature is not active or it is inapplicable (camera only mode / 2D)
        INACTIVE,

        ///Correction (and optionally also the stabilization) was computed and applied
        CORRECTION_APPLIED,
        ///A new reference was successfully recorded
        REFERENCE_SUCCESSFULLY_RECORDED,

        ///Correction attempted, but not enough dots recognized; correction from the previous scan applied
        NOT_ENOUGH_DOTS_RECOGNIZED,
        ///Correction attempted, but recognized dots span low projection range; correction from the previous scan applied
        INSUFFICIENT_PROJECTION_COVERAGE,
        ///Correction attempted, but some dot is very close to the projection boundary; correction from the previous scan applied
        MARKER_DOT_AT_THE_PROJECTION_BOUNDARY,
        ///Correction attempted, but some dot was displaced more than allowed; correction from the previous scan applied
        MARKER_DOT_MOTION_DETECTED,
        ///Correction attempted, but the recognized dots are collinear, which prohibits the coordinate stabilization;
        ///correction from the previous scan applied
        COLLINEAR_MARKER_DOTS,
        ///Correction attempted, but the correction model limitation was reached; correction from the previous scan applied
        ///Either the relative position of the dots in the scene has changed or a maintence of the device is required
        CORRECTION_MODEL_LIMITATION_REACHED,

        ///Correction attempted, but reference does not exist
        NON_EXISTENT_REFERENCE,
        ///Reference recording attempted, but failed either due to low count of recognized dots or due to low projection coverage
        REFERENCE_RECORDING_FAILURE,
        ///The selected capturing mode is not supported (applies for Raw and Irregular topology in Camera mode)
        UNSUPPORTED_CAPTURING_MODE,
        ///The calibration of the device has changed, a reference needs to be recorded
        INVALIDATED_REFERENCE,
        ///Unexpected internal error, more details in the message
        UNEXPECTED_ERROR
    };
    struct MarkerDotPosition {
        /// Pixel position of the center of the corresponding reference marker dot,
        /// appropriately recomputed to the current acquisition resolution
        Point2_32f reference2d;
        /// 3D coordinate of the corresponding reference marker dot
        /// in the CameraSpace coordinates
        Point3_32f reference3d;
        ///Pixel position of the center of a recognized marker dot
        Point2_32f observation2d;
        ///3D coordinate of the recognized marker dot without application of the correction
        ///in the CameraSpace coordinates
        Point3_32f observation3d;
        ///3D coordinate of the recognized marker dot with application of the correction
        ///in the CameraSpace coordinates
        Point3_32f corrected3d;
    };

    ///Marker dot correction status
    MarkerDotStatus Status;
    ///Marker dot correction status message
    std::string Message;
    ///Pixel positions of the centers of recognized marker dots
    std::vector<MarkerDotPosition> RecognizedMarkerDots;
    operator MarkerDotStatus() const { return Status; };
    operator std::string() const { return Message; }
};

struct PHOXI_DLL_API PhoXiPTPTime {
    class Clock {
    public:
        typedef std::chrono::nanoseconds duration;
        typedef duration::rep rep;
        typedef duration::period period;
        typedef std::chrono::time_point<Clock> time_point;

        static time_point now() PHO_API_NOEXCEPT {
            using namespace std::chrono;
            return time_point(duration_cast<duration>(seconds(0) + nanoseconds(0)));
        }

        static std::time_t to_time_t(const time_point &t) PHO_API_NOEXCEPT {
            return std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch()).count();
        }

        static time_point from_time_t(std::time_t t) PHO_API_NOEXCEPT {
            typedef std::chrono::time_point<Clock, std::chrono::seconds> from;
            return std::chrono::time_point_cast<Clock::duration>(from(std::chrono::seconds(t)));
        }
    };

    ///Time point without leap seconds
    Clock::time_point Time;
    std::string PortState;
    std::string GrandMasterIdentity;

    bool IsValid() const { return Time.time_since_epoch().count() > 0; }

    ///Format time to the string by standard format from the strftime
    std::string TimeAsString(const std::string &format) const;
};

#pragma pack(push, 8)
///Additional frame informations.
class PHOXI_DLL_API FrameInfo {
public:
    ///Frame index counted from the connection start.
    uint64_t FrameIndex;
    ///Frame start time in ms counted from the connection start.
    double FrameTimestamp;
    ///Frame capturing duration in ms.
    double FrameDuration;
    ///Frame computation duration in ms.
    double FrameComputationDuration;
    ///Frame transfer duration in ms.
    double FrameTransferDuration;
    ///Frame start acquisition time point synchronized by PTP
    PhoXiPTPTime FrameStartTime;
    ///Sensor position in the 3D space.
    Point3_64f SensorPosition;
    ///Sensor 3D Axis in the space.
    Point3_64f SensorXAxis, SensorYAxis, SensorZAxis;
    ///Total device scan count.
    int64_t TotalScanCount;
    ///Filename path for the file camera only, empty for the physical device
    std::string FilenamePath;
    ///Device ID
    std::string HWIdentification;
    ///Array of temperatures from various sensors.
    std::array<double, TemperatureDeviceSensor::TempsCount> Temperatures;
    ///Camera's 3x3 matrix describing the mapping from 3D (world) to 2D (image)
    ///corresponding to the current scanning mode.
    ///Note that the camera matrix is going to be empty in modes with non-regular
    ///point cloud topology, as there the standard camera matrix model does not hold.
    CameraMatrix64f CameraMatrix;
    ///Camera's distortion coefficients. It's using OpenCV format:
    ///(k1, k2, p1, p2[, k3[, k4, k5, k6[, s1, s2, s3, s4[, tx, ty]]]]).
    std::vector<double> DistortionCoefficients;
    ///Camera pixels binning
    PhoXiSize CameraBinning;
    ///White balance RGB coefficients
    Point3_64f BalanceRGB;
    ///Color camera sensor position in 3D space.
    Point3_64f ColorCameraPosition;
    ///Color camera sensor 3D axis in space.
    Point3_64f ColorCameraXAxis, ColorCameraYAxis, ColorCameraZAxis;
    ///Color camera's 3x3 matrix describing the mapping from 3D (world) to 2D (image)
    ///corresponding to the current scanning mode.
    ///Note that the camera matrix is going to be empty in modes with non-regular
    ///point cloud topology, as there the standard camera matrix model does not hold.
    CameraMatrix64f ColorCameraMatrix;
    ///Color camera's distortion coefficients. It's using OpenCV format:
    ///(k1, k2, p1, p2[, k3[, k4, k5, k6[, s1, s2, s3, s4[, tx, ty]]]]).
    std::vector<double> ColorCameraDistortionCoefficients;
    ///Color camera pixels scaling
    PhoXiSize_64f ColorCameraScale;
    ///Marker dot correction information
    PhoXiMarkerDots MarkerDots;
    ///Current camera position in 3D space.
    Point3_64f CurrentCameraPosition;
    ///Current camera 3D axis in space.
    Point3_64f CurrentCameraXAxis, CurrentCameraYAxis, CurrentCameraZAxis;

    FrameInfo();
};
#pragma pack(pop)

#pragma pack(push, 8)
///FrameMatHolder is a helper class that stores shared_ptr objects owned by Frame 
class FrameMatHolder {
protected:
    PPointCloud32f PointCloudPtr;
    PNormalMap32f NormalMapPtr;
    PDepthMap32f DepthMapPtr;
    PConfidenceMap32f ConfidenceMapPtr;
    PTexture32f TexturePtr;
    PTextureRGB16 TextureRGBPtr;
    PIndexEventMap32f EventMapPtr;
    PTextureRGB16 ColorCameraImagePtr;
public:
    FrameMatHolder()
        : PointCloudPtr(std::make_shared<PointCloud32f>())
        , NormalMapPtr(std::make_shared<NormalMap32f>())
        , DepthMapPtr(std::make_shared<DepthMap32f>())
        , ConfidenceMapPtr(std::make_shared<ConfidenceMap32f>())
        , TexturePtr(std::make_shared<Texture32f>())
        , TextureRGBPtr(std::make_shared<TextureRGB16>())
        , EventMapPtr(std::make_shared<EventMap32f>())
        , ColorCameraImagePtr(std::make_shared<TextureRGB16>()) {}
};
///Frame output structure.
class Frame : public FrameMatHolder {
public:
    bool Successful;
    ///Additional informations.
    FrameInfo Info;
    /**
    2 dimensional organized 3 channel PointCloud structure, organized as
    [X mm, Y mm, Z mm] of 32 bit floats, unmeasured point has [0.0, 0.0, 0.0].
    */
    PointCloud32f& PointCloud;
    /**
    2 dimensional organized 3 channel NormalMap structure, organized as normalized
    [X, Y, Z] of 32 bit floats, unmeasured normal has [0.0, 0.0, 0.0].
    */
    NormalMap32f& NormalMap;
    /**
    2 dimensional 1 channel Depth map of 32 bit floats coding orthogonal distances
    from the internal camera in mm.
    */
    DepthMap32f& DepthMap;
    /**
    2 dimensional 1 channel Confidence map of 32 bit floats coding measurement
    confidence.
    */
    ConfidenceMap32f& ConfidenceMap;
    /**
    2 dimensional 1 channel Event map of 32 bit floats coding the time of measurement after the scan trigger.
    This map is available only for Motion cam in Camera mode, otherwise map is empty.
    */
    EventMap32f& EventMap;
    ///2 dimensional 1 channel Texture of 32 bit floats coding intensity.
    Texture32f& Texture;
    ///2 dimensional 3 channel Texture of 16 bit unsigned integers coding RGB values.
    TextureRGB16& TextureRGB;
    ///2 dimensional 3 channel Image of 16 bit unsigned integers coding RGB values from color camera.
    TextureRGB16& ColorCameraImage;
    ///Contains content of a CustomMessage assigned on TriggerFrame.
    std::string CustomMessage;
    /**
    Contains message about frame state, empty when frame was created and received successfully.
    @since API version 1.6.0
    */
    std::string StatusMessage;
    ///Returns true if there is no output, returns false otherwise.
    bool PHOXI_DLL_API Empty() const;
    /**
    Returns resolution of the captured frame - All outputs are either empty,
    or have this resolution.
    */
    PhoXiSize PHOXI_DLL_API GetResolution() const;
    /**
    Creates a shared_ptr of Frame with new allocated data in the local
    address space.
    */
    std::shared_ptr<Frame> Clone() {
        std::shared_ptr<Frame> Result = std::make_shared<Frame>();
        Result->Successful = Successful;
        Result->Info = Info;
        Result->PointCloud = PointCloud;
        Result->NormalMap = NormalMap;
        Result->DepthMap = DepthMap;
        Result->ConfidenceMap = ConfidenceMap;
        Result->EventMap = EventMap;
        Result->Texture = Texture;
        Result->TextureRGB = TextureRGB;
        Result->CustomMessage = CustomMessage;
        Result->StatusMessage = StatusMessage;
        return Result;
    }
    ///Save the Frame as an Ply with all satellite data
    /**
    Saves Frame structure as an organized Stanford's PLY with specified Texture
    and Normals. All informations are stored, this can be used as an container.
    @param FilePath Input ply FilePath.
    @param BinaryFile If true, the ply will be stored in binary form, more
    memory efficient (true is default).
    @param NormalizeTexture If true, the RGB information in the PLY will be
    normalized, 32 bit Intensity information will be untouched.
    @param StorePointCloud If true and PointCloud is not empty, the PointCloud
    will be stored into PLY.
    @param StoreNormalMap If true and NormalMap is not empty, the NormalMap
    will be stored into PLY.
    @param StoreDepthMap If true and DepthMap is not empty, the DepthMap will
    be stored into PLY.
    @param StoreTexture If true and Texture is not empty, the Texture will be
    stored into PLY.
    @param StoreConfidenceMap If true and ConfidenceMap is not empty, the
    ConfidenceMap will be stored into PLY.
    @param Unordered If true the Frame will be stored without non-measured
    points.
    @param Metadata If true, every available information will be stored into
    PLY. If false, only vertex elements will be present in saved PLY file
    (HALCON compatible).
    @return true on success.
    */
    bool PHOXI_DLL_API SaveAsPly(
        const std::string &FilePath,
        bool BinaryFile = true,
        bool NormalizeTexture = true,
        bool StorePointCloud = true,
        bool StoreNormalMap = true,
        bool StoreDepthMap = true,
        bool StoreTexture = true,
        bool StoreConfidenceMap = true,
        bool Unordered = false,
        bool Metadata = true,
        bool StoreEventMap = false) const;


    /**
    Saves Frame structure as an organized Stanford's PLY with specified
    Texture and Normals. All informations are stored, this can be used
    as an container.
    @param FilePath Input ply FilePath.
    @param BinaryFile If true, the ply will be stored in binary form, more
    memory efficient (true is default).
    @param NormalizeTexture If true, the RGB information in the PLY will be
    normalized, 32 bit Intensity information will be untouched.
    @param StorePointCloud If true and PointCloud is not empty, the PointCloud
    will be stored into PLY.
    @param StoreNormalMap If true and NormalMap is not empty, the NormalMap
    will be stored into PLY.
    @param StoreDepthMap If true and DepthMap is not empty, the DepthMap will
    be stored into PLY.
    @param StoreTexture If true and Texture is not empty, the Texture will be
    stored into PLY.
    @param StoreConfidenceMap If true and ConfidenceMap is not empty, the
    ConfidenceMap will be stored into PLY.
    @param Unordered If true the Frame will be stored without non-measured
    points.
    @param cameraCalibrationMatrix std::vector<float>(9).
    @param distortionCoefficientsMatrix std::vector<float>(14).
    @param cameraResolution std::vector<float>(2).
    @param cameraBinning std::vector<float>(2).
    @param Metadata If true, every available information will be stored into
    PLY. If false, only vertex elements will be present in saved PLY file
    (HALCON compatible).
    @return true on success.
    */
    bool PHOXI_DLL_API SaveAsPly(
        const std::wstring &FilePath,
        bool BinaryFile = true,
        bool NormalizeTexture = true,
        bool StorePointCloud = true,
        bool StoreNormalMap = true,
        bool StoreDepthMap = true,
        bool StoreTexture = true,
        bool StoreConfidenceMap = true,
        bool Unordered = false,
        const std::vector<float> &CameraCalibrationMatrix = std::vector<float>(),
        const std::vector<float> &DistortionCoefficientsMatrix = std::vector<float>(),
        const std::vector<float> &CameraResolution = std::vector<float>(),
        const std::vector<float> &CameraBinning = std::vector<float>(),
        bool Metadata = true,
        bool StoreEventMap = false) const;

    Frame()
        : Successful(false)
        , PointCloud(*PointCloudPtr)
        , NormalMap(*NormalMapPtr)
        , DepthMap(*DepthMapPtr)
        , ConfidenceMap(*ConfidenceMapPtr)
        , EventMap(*EventMapPtr)
        , Texture(*TexturePtr)
        , TextureRGB(*TextureRGBPtr)
        , ColorCameraImage(*ColorCameraImagePtr) {}

#ifdef PHOXI_PCL_SUPPORT
    ///Converts to PCL's PointCloud structure
    /**
    Based on the chosen type, the data will be present in the output
    The data are copied
    */
    template <class PCLPointType>
    bool ConvertTo(pcl::PointCloud<PCLPointType>& Destination, bool NormalizeTexture = true) const;
#  ifdef PCL_SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
    ///Converts to PCL's obsolete PointCloud2 structure
    bool ConvertTo(pcl::PCLPointCloud2& Destination) const;
#  endif
#endif
private:
    bool canSavePLY(bool savePointCloud,
                    bool saveNormalMap,
                    bool saveDepthMap,
                    bool saveTexture,
                    bool saveConfidenceMap,
                    bool saveEventMap) const;

};
#pragma pack(pop)
typedef std::shared_ptr <Frame> PFrame;

#pragma pack(push, 8)
class PHOXI_DLL_API PhoXiWhiteBalance {
public:
    bool Enabled;
    std::string Preset;
    Point3_64f BalanceRGB;
    bool ComputeCustomWhiteBalance;
    PhoXiWhiteBalance& operator=(const PhoXiWhiteBalance &Other);
    bool operator==(const PhoXiWhiteBalance &Other) const;
};
#pragma pack(pop)

} // api
} // pho

#ifndef PHOXI_API_ENABLE_W4251
#pragma warning(pop)
#endif

#endif //_PHOXI_DATA_TYPES_H
