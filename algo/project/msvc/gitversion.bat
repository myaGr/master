@echo off

set VERSION_FILE=version.in

REM 获取git branch name
for /F %%i in ('git rev-parse --abbrev-ref HEAD') do ( set GIT_BRANCH=%%i)

REM 获取git commit id
for /F %%i in ('git rev-parse --short HEAD') do ( set GIT_COMMIT_ID=%%i)

DEL /f %VERSION_FILE%
echo /D GIT_BRANCH=\"%GIT_BRANCH%\" >> %VERSION_FILE%
echo /D GIT_COMMIT_ID=\"%GIT_COMMIT_ID%\" >> %VERSION_FILE%