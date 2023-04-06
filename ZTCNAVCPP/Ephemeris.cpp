#include "Ephemeris.h"

double Ephemeris::GetE_k(const double& M_k) const
{
	auto E_k{ M_k }, temp{ M_k + Eccentricity * sin(E_k) };
	auto i{ 0 };
	while (abs(E_k - temp) > 1E-15 && i < 10)
	{
		E_k = temp;
		temp = M_k + Eccentricity * sin(E_k);
		i++;
	}
	return E_k;
}

CartCoordinate Ephemeris::Position(const GpsTime& t) const
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
	auto Omega_k{ RightAscension + (RateOfRightAscension - Omega_e()) * t_k - Omega_e() * ReferenceTimeForEphemeris.Second() };
	auto sinOmega_k{ sin(Omega_k) }, cosOmega_k{ cos(Omega_k) };
	return CartCoordinate
	{
		x_k * cosOmega_k - y_k * cos(i_k) * sinOmega_k,
		x_k * sinOmega_k + y_k * cos(i_k) * cosOmega_k,
		y_k * sin(i_k)
	};
}

double Ephemeris::ClockError(const GpsTime& t) const
{
	auto n_0{ sqrt(GM() / pow(Semi_majorAxis, 3)) };
	auto t_k{ t - ReferenceTimeForEphemeris };
	auto n{ n_0 + MeanAngularVelocityDifference };
	auto M_k{ MeanAnomalyOfReferenceTime + n * t_k };
	t_k = t - ReferenceTimeForSatelliteClockCorrction;
	return ClockJitter + ClockWander * (t_k)+ClockWanderSpeed * pow(t_k, 2) + F() * Eccentricity * sqrt(Semi_majorAxis) * sin(GetE_k(M_k));
}

Vector Ephemeris::Velocity(const GpsTime& t) const
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
	auto delta_u_k{ Sin_CorrectionArgumentOfLatitude * sin2Phi_k + Cos_CorrectionArgumentOfLatitude * cos2Phi_k };
	auto delta_r_k{ Sin_CorrectionOrbitRadius * sin2Phi_k + Cos_CorrectionOrbitRadius * cos2Phi_k };
	auto delta_i_k{ Sin_CorrectionAngleOfInclination * sin2Phi_k + Cos_CorrectionAngleOfInclination * cos2Phi_k };
	auto i_k{ InclinationAngleAtReferenceTime + delta_i_k + RateOfInclinationAngle * t_k };
	auto u_k{ Phi_k + delta_u_k };
	auto dot_i_k{ RateOfInclinationAngle + 2 * dot_v_k * (Sin_CorrectionAngleOfInclination * cos2Phi_k - Cos_CorrectionAngleOfInclination * sin2Phi_k) };
	auto dot_u_k{ dot_v_k + 2 * dot_v_k * (Sin_CorrectionArgumentOfLatitude * cos2Phi_k - Cos_CorrectionArgumentOfLatitude * sin2Phi_k) };
	auto dot_r_k{ Eccentricity * Semi_majorAxis * dot_Ek * sinE_k + 2 * dot_v_k * (Sin_CorrectionOrbitRadius * cos2Phi_k - Cos_CorrectionOrbitRadius * sin2Phi_k) };
	auto dot_Omega_k{ RateOfRightAscension - Omega_e() };
	auto r_k{ Semi_majorAxis * (1 - Eccentricity * cosE_k) + delta_r_k };
	auto dot_x_k{ dot_r_k * cos(u_k) - r_k * dot_u_k * sin(u_k) };
	auto dot_y_k{ dot_r_k * sin(u_k) + r_k * dot_u_k * cos(u_k) };
	auto Omega_k{ RightAscension + (RateOfRightAscension - Omega_e()) * t_k - Omega_e() * ReferenceTimeForEphemeris.Second() };
	auto sinOmega_k = sin(Omega_k), cosOmega_k = cos(Omega_k);
	auto sini_k{ sin(i_k) }, cosi_k{ cos(i_k) };
	auto x_k{ r_k * cos(u_k) };
	auto y_k{ r_k * sin(u_k) };
	return Vector
	{
		{
			-x_k * dot_Omega_k * sinOmega_k + dot_x_k * cosOmega_k - dot_y_k * sinOmega_k * cosi_k - y_k * (dot_Omega_k * cosOmega_k * cosi_k - dot_i_k * sinOmega_k * sini_k),
			 x_k * dot_Omega_k * cosOmega_k + dot_x_k * sinOmega_k + dot_y_k * cosOmega_k * cosi_k - y_k * (dot_Omega_k * sinOmega_k * cosi_k + dot_i_k * cosOmega_k * sini_k),
			 dot_y_k * sini_k + y_k * dot_i_k * cosi_k
		}
	};
}

double Ephemeris::ClockSpeedError(const GpsTime& t) const
{
	auto n_0{ sqrt(GM() / pow(Semi_majorAxis, 3)) };
	auto t_k{ t - ReferenceTimeForEphemeris };
	auto n{ n_0 + MeanAngularVelocityDifference };
	auto M_k{ MeanAnomalyOfReferenceTime + n * t_k };
	auto E_k{ GetE_k(M_k) };
	auto sinE_k{ sin(E_k) }, cosE_k{ cos(E_k) };
	auto dot_Ek{ n / (1 - Eccentricity * cosE_k) };
	return ClockWander + 2 * ClockWanderSpeed * t_k + F() * Eccentricity * sqrt(Semi_majorAxis) * cosE_k * dot_Ek;
}