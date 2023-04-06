#include "OEMBDSEPHEMRIS.h"

OEMBDSEPHEMRIS::OEMBDSEPHEMRIS(const OEMMessageHeader& hd) :Header(hd) {}

CartCoordinate OEMBDSEPHEMRIS::Position(const GpsTime& t) const noexcept
{	
	auto n_0{ sqrt(GM() / pow(Semi_majorAxis, 3)) };
	auto t_k{ t - ReferenceTimeForEphemeris };
	auto n{ n_0 + MeanAngularVelocityDifference };
	auto M_k{ MeanAnomalyOfReferenceTime + n * t_k };
	auto E_k{ GetE_k(M_k) };
	auto sinE_k{ sin(E_k) }, cosE_k{ cos(E_k) };
	auto v_k{ atan2(sqrt(1 - pow(Eccentricity, 2)) * sinE_k / (1 - Eccentricity * cosE_k), (cosE_k - Eccentricity) / (1 - Eccentricity * cosE_k)) };
	auto Phi_k{ v_k + ArgumentOfPerigee };
	auto sin2Phi_k{ sin(2 * Phi_k) };
	auto cos2Phi_k{ cos(2 * Phi_k) };
	auto delta_u_k{ Sin_CorrectionArgumentOfLatitude * sin2Phi_k + Cos_CorrectionArgumentOfLatitude * cos2Phi_k };
	auto delta_r_k{ Sin_CorrectionOrbitRadius * sin2Phi_k + Cos_CorrectionOrbitRadius * cos2Phi_k };
	auto delta_i_k{ Sin_CorrectionAngleOfInclination * sin2Phi_k + Cos_CorrectionAngleOfInclination * cos2Phi_k };
	auto u_k{ Phi_k + delta_u_k };
	auto r_k{ Semi_majorAxis * (1 - Eccentricity * cosE_k) + delta_r_k };
	auto i_k{ InclinationAngleAtReferenceTime + delta_i_k + RateOfInclinationAngle * t_k };
	auto x_k{ r_k * cos(u_k) };
	auto y_k{ r_k * sin(u_k) };	
	if (!SatelliteOfEphemeris.IsGEO())
	{
		auto Omega_k{ RightAscension + (RateOfRightAscension - Omega_e()) * t_k - Omega_e() * (ReferenceTimeForEphemeris.Second() - 14) };
		auto sinOmega_k{ sin(Omega_k) }, cosOmega_k{ cos(Omega_k) };
		return CartCoordinate
		{
			x_k * cosOmega_k - y_k * cos(i_k) * sinOmega_k,
			x_k * sinOmega_k + y_k * cos(i_k) * cosOmega_k,
			y_k * sin(i_k)
		};
	}
	else
	{
		auto Omega_k{ RightAscension + RateOfRightAscension * t_k - Omega_e() * (ReferenceTimeForEphemeris.Second() - 14) };
		auto sinOmega_k{ sin(Omega_k) }, cosOmega_k{ cos(Omega_k) };
		auto X_GK{ x_k * cosOmega_k - y_k * cos(i_k) * sinOmega_k };
		auto Y_GK{ x_k * sinOmega_k + y_k * cos(i_k) * cosOmega_k };
		auto Z_GK{ y_k * sin(i_k) };
		auto sinOmega_et_k{ sin(Omega_e() * t_k) }, cosOmega_et_k{ cos(Omega_e() * t_k) };
		auto sinphi{ sin(Angle::ToRad(-5.0)) }, cosphi{ cos(Angle::ToRad(-5.0)) };
		return CartCoordinate
		{
			cosOmega_et_k * X_GK + sinOmega_et_k * (cosphi * Y_GK + sinphi * Z_GK),
			-sinOmega_et_k * X_GK + cosOmega_et_k * (cosphi * Y_GK + sinphi * Z_GK),
			-sinphi * Y_GK + cosphi * Z_GK
		};
	}	
}

Vector OEMBDSEPHEMRIS::Velocity(const GpsTime& t) const noexcept
{
	auto n_0{ sqrt(GM() / pow(Semi_majorAxis, 3)) };
	auto t_k{ t - ReferenceTimeForEphemeris };
	auto n{ n_0 + MeanAngularVelocityDifference };
	auto M_k{ MeanAnomalyOfReferenceTime + n * t_k };
	auto E_k{ GetE_k(M_k) };
	auto sinE_k{ sin(E_k) }, cosE_k{ cos(E_k) };
	auto v_k{ atan2(sqrt(1 - pow(Eccentricity, 2)) * sinE_k / (1 - Eccentricity * cosE_k), (cosE_k - Eccentricity) / (1 - Eccentricity * cosE_k)) };
	auto Phi_k{ v_k + ArgumentOfPerigee };
	auto sin2Phi_k{ sin(2 * Phi_k) };
	auto cos2Phi_k{ cos(2 * Phi_k) };
	auto dot_Ek{ n / (1 - Eccentricity * cosE_k) };
	auto dot_v_k{ dot_Ek * sqrt(1 - pow(Eccentricity,2)) / (1 - Eccentricity * cosE_k) };
	auto dot_i_k{ RateOfInclinationAngle + 2 * dot_v_k * (Sin_CorrectionAngleOfInclination * cos2Phi_k - Cos_CorrectionAngleOfInclination * sin2Phi_k) };
	auto dot_u_k{ dot_v_k + 2 * dot_v_k * (Sin_CorrectionArgumentOfLatitude * cos2Phi_k - Cos_CorrectionArgumentOfLatitude * sin2Phi_k) };
	auto dot_r_k{ Eccentricity * Semi_majorAxis * dot_Ek * sinE_k + 2 * dot_v_k * (Sin_CorrectionOrbitRadius * cos2Phi_k - Cos_CorrectionOrbitRadius * sin2Phi_k) };
	auto dot_Omega_k{ RateOfRightAscension - Omega_e() };
	auto delta_u_k{ Sin_CorrectionArgumentOfLatitude * sin2Phi_k + Cos_CorrectionArgumentOfLatitude * cos2Phi_k };
	auto delta_r_k{ Sin_CorrectionOrbitRadius * sin2Phi_k + Cos_CorrectionOrbitRadius * cos2Phi_k };
	auto delta_i_k{ Sin_CorrectionAngleOfInclination * sin2Phi_k + Cos_CorrectionAngleOfInclination * cos2Phi_k };
	auto r_k{ Semi_majorAxis * (1 - Eccentricity * cosE_k) + delta_r_k };
	auto i_k{ InclinationAngleAtReferenceTime + delta_i_k + RateOfInclinationAngle * t_k };
	auto u_k{ Phi_k + delta_u_k };
	auto sinu_k{ sin(u_k) }, cosu_k{ cos(u_k) };
	auto dot_x_k{ dot_r_k * cosu_k - r_k * dot_u_k * sinu_k };
	auto dot_y_k{ dot_r_k * sinu_k + r_k * dot_u_k * cosu_k };
	auto sini_k{ sin(i_k) }, cosi_k{ cos(i_k) };
	auto x_k{ r_k * cos(u_k) };
	auto y_k{ r_k * sin(u_k) };
	if (!SatelliteOfEphemeris.IsGEO())
	{		
		auto Omega_k{ RightAscension + (RateOfRightAscension - Omega_e()) * t_k - Omega_e() * (ReferenceTimeForEphemeris.Second() - 14) };
		auto sinOmega_k = sin(Omega_k), cosOmega_k = cos(Omega_k);		
		return Vector
		{
			{
				-x_k * dot_Omega_k * sinOmega_k + dot_x_k * cosOmega_k - dot_y_k * sinOmega_k * cosi_k - y_k * (dot_Omega_k * cosOmega_k * cosi_k - dot_i_k * sinOmega_k * sini_k),
				 x_k * dot_Omega_k * cosOmega_k + dot_x_k * sinOmega_k + dot_y_k * cosOmega_k * cosi_k - y_k * (dot_Omega_k * sinOmega_k * cosi_k + dot_i_k * cosOmega_k * sini_k),
				 dot_y_k * sini_k + y_k * dot_i_k * cosi_k
			}
		};
	}
	else
	{		
		auto Omega_k{ RightAscension + RateOfRightAscension * t_k - Omega_e() * (ReferenceTimeForEphemeris.Second() - 14) };
		auto sinOmega_k{ sin(Omega_k) }, cosOmega_k{ cos(Omega_k) };
		auto X_GK{ x_k * cosOmega_k - y_k * cosi_k * sinOmega_k };
		auto Y_GK{ x_k * sinOmega_k + y_k * cosi_k * cosOmega_k };
		auto Z_GK{ y_k * sini_k };
		auto sinOmega_et_k{ sin(Omega_e() * t_k) }, cosOmega_et_k{ cos(Omega_e() * t_k) };
		auto sinphi{ sin(Angle::ToRad(-5.0)) }, cosphi{ cos(Angle::ToRad(-5.0)) };
		auto dot_X_GK{ dot_x_k * cosOmega_k - x_k * sinOmega_k * RateOfRightAscension - dot_y_k * cosi_k * sinOmega_k + y_k * (sini_k * dot_i_k * sinOmega_k - cosi_k * cosOmega_k * RateOfRightAscension) };
		auto dot_Y_GK{ dot_x_k * sinOmega_k + x_k * cosOmega_k * RateOfRightAscension + dot_y_k * cosi_k * cosOmega_k - y_k * (sini_k * dot_i_k * cosOmega_k + cosi_k * sinOmega_k * RateOfRightAscension) };
		auto dot_Z_GK{ dot_y_k * sini_k + y_k * cosi_k * dot_i_k };
		auto dot_sinOmega_et_k{ Omega_e() * cosOmega_et_k };
		auto dot_cosOmega_et_k{ -Omega_e() * sinOmega_et_k };
		return Vector{
			{
				dot_cosOmega_et_k * X_GK + cosOmega_et_k * dot_X_GK + dot_sinOmega_et_k * (cosphi * Y_GK + sinphi * Z_GK) + sinOmega_et_k * (cosphi * dot_Y_GK + sinphi * dot_Z_GK),
				- dot_sinOmega_et_k * X_GK - sinOmega_et_k * dot_X_GK + dot_cosOmega_et_k * (cosphi * Y_GK + sinphi * Z_GK) + cosOmega_et_k * (cosphi * dot_Y_GK + sinphi * dot_Z_GK),
				- sinphi * dot_Y_GK + cosphi * dot_Z_GK
			}
		};
	}	
}
