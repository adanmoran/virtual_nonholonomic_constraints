:: This batch file copies the relevant source files to the main folder
:: so that Arduino can compile the SUGAR software
@ECHO OFF
:: Get the path of the batch file so we can go there regardless of where the
:: batch file is started from
SET batch_path=%~dp0
ECHO batch_path
:: Go to that folder
PUSHD "%batch_path%"
:: Now go up a folder, which is the root of the SUGAR library
cd ..
SET SUGAR_ROOT=%CD%
:: and copy all the required files to this root folder
COPY /Y "%SUGAR_ROOT%\src\VNHC.cpp" "%SUGAR_ROOT%\VNHC.cpp"
COPY /Y "%SUGAR_ROOT%\src\Phase.cpp" "%SUGAR_ROOT%\Phase.cpp"
COPY /Y "%SUGAR_ROOT%\src\AcrobotDynamics.cpp" "%SUGAR_ROOT%\AcrobotDynamics.cpp"
COPY /Y "%SUGAR_ROOT%\src\Supervisor.cpp" "%SUGAR_ROOT%\Supervisor.cpp"
DIR
ECHO Please verify Supervisor.cpp, VNHC.cpp, Phase.cpp, and AcrobotDynamics.cpp were copied correctly.
PAUSE