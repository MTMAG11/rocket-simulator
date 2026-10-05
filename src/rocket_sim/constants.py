"""Physical constants (SI units)."""

G0 = 9.80665  # standard gravity [m/s^2] (defines Isp conventions; BIPM/CGPM 1901)
GM_EARTH = 3.986004418e14  # Earth gravitational parameter [m^3/s^2] (WGS-84)
R_EARTH = 6_371_008.8  # mean Earth radius [m] (IUGG)
R_AIR = 287.05287  # specific gas constant of dry air [J/(kg K)] (U.S. Std Atm. 1976)
GAMMA_AIR = 1.4  # ratio of specific heats, dry air
T_SEA_LEVEL = 288.15  # ISA sea-level temperature [K]
P_SEA_LEVEL = 101_325.0  # ISA sea-level pressure [Pa]
SUTHERLAND_MU0 = 1.458e-6  # Sutherland viscosity coefficient [kg/(m s K^0.5)]
SUTHERLAND_S = 110.4  # Sutherland temperature [K]
EARTH_MAG_FIELD_ENU_T = (0.0, 2.0e-5, -4.0e-5)  # representative mid-latitude field [T], ENU
