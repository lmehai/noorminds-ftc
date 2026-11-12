# NoorMinds start up


## Education Videos
### Introduction:
> https://www.youtube.com/watch?v=kOrykopGAh0
> https://www.youtube.com/watch?v=JEvdoWbNBBg&list=PLfZDz4HU7SLzsM5ukVJbVSyTZdQdoVyuK&index=4

### Java Programming:
> https://www.youtube.com/watch?v=fbSiZSnPf-U&t=15s


### Block Programming (Just for understanding)


# Add Android Debug Bridge (ADB) to windows path
Run the following from Power shell on time
> & "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe"


## Connect to the Control Hub using WIFI 
> adb connect 192.168.43.1:5555

## Verify connection to Control Hub
adb devices


## Build Command using gradle
> .\gradlew.bat :TeamCode:assembleDebug

## Deploy the Android Package Kit (apk) to Control Hub
> .\gradlew.bat :TeamCode:installDebug


## uninstall  FTC APK (Clean up)
> adb uninstall com.qualcomm.ftcrobotcontroller 



