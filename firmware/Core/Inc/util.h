#pragma once

#define ERROR_DEF HAL_StatusTypeDef res
#define CHECK(val) if ((res = (val)) != HAL_OK) return res
