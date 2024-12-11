#ifndef ADDITIONAL_CAMERA_H
#define ADDITIONAL_CAMERA_H

#ifndef PHOXI_API_ENABLE_W4251
#pragma warning(push)
#pragma warning(disable:4251)
#endif

#include "PhoXi.h"
#include "PhoXiDataTypes.h"
#include "PhoXiFeatureTypes.h"

namespace pho {
	namespace api {
		namespace AdditionalCamera {
			class PHOXI_DLL_API Calibrator {
			public:
				Calibrator(const PPhoXi& PhoXiDevice);
				~Calibrator();
				bool Calibrate(
					const std::vector<Texture32f> &Frames, 
					const std::vector<Texture32f> &Images, 
					double FocalLength, 
					double PixelSize, 
					const std::string &MarkersPositions, 
					AdditionalCameraCalibration &Calibration) const;
			protected:
				class Impl;
			private:
				std::unique_ptr<Impl> _Impl;
			};

			class PHOXI_DLL_API Aligner {
			public:
				Aligner(const PPhoXi& PhoXiDevice, const AdditionalCameraCalibration& Calibration);
				~Aligner();
				bool GetAlignedDepthMap(DepthMap32f &DepthMap) const;
			protected:
				class Impl;
			private:
				std::unique_ptr<Impl> _Impl;
			};
		}
	}
}

#ifndef PHOXI_API_ENABLE_W4251
#pragma warning(pop)
#endif

#endif