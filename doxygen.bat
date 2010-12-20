rem delete previous content
for /d %%i in ("doc\*") do rmdir /s /q "%%i"

rem generate documentation
doxygen doxyconf