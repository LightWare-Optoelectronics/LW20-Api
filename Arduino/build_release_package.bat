rmdir /s /q "release"

xcopy "source" "release\LW20\" /s /i /y
xcopy "examples" "release\LW20\examples" /s /i /y
copy "..\lw20api.h" "release\LW20\lw20api.h" /y

pause