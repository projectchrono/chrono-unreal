static inline float Normalize(const float Value, const float Min, const float Max)
{
	if(Min == Max) return 0.0;

	return (Value - Min) / (Max - Min);
}

static inline double Normalize(const double Value, const double Min, const double Max)
{
	if (Min == Max) return 0.0;

	return (Value - Min) / (Max - Min);
}

static inline float MapRange(const float InputMin, const float InputMax, const float OutputMin, const float OutputMax, const float Value)
{
	const float Normalized = (Value - InputMin) / (InputMax - InputMin);
	const float MappedValue = OutputMin + (OutputMax - OutputMin) * Normalized;
	if(MappedValue < OutputMin) return OutputMin;
	else if(MappedValue > OutputMax) return OutputMax;
	else return MappedValue;
}

//Assumes input range is already normalized between 0-1
static inline float MapRangeFast(const float InputMin, const float InputMax, const float OutputMin, const float OutputMax, const float Value)
{	
	const float MappedValue = OutputMin + (OutputMax - OutputMin) * Value;
	if (MappedValue < OutputMin) return OutputMin;
	else if (MappedValue > OutputMax) return OutputMax;
	else return MappedValue;
}

//Map range without using branching
static inline float MapRangeFastNoBranch(const float InputMin, const float InputMax, const float OutputMin, const float OutputMax, const float Value)
{
	const float MappedValue = OutputMin + (OutputMax - OutputMin) * Value;
	const float clampedValue = MappedValue - OutputMin;
	const float clampedValueNeg = -clampedValue;

	const int maskBelow = *(int*)&clampedValueNeg >> 31; // 0xFFFFFFFF if clampedValueNeg < 0, 0x00000000 otherwise
	const int maskAbove = *(int*)&clampedValue >> 31;     // 0xFFFFFFFF if clampedValue >= 0, 0x00000000 otherwise

	const float result = MappedValue - (clampedValue * (float)maskBelow) + (OutputMax * (float)maskBelow);
	return result * (float)maskAbove + OutputMin * (float)maskAbove;
}
