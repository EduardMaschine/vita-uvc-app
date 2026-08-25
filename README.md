# First words
The whole development was chaos.
I have no structure. The whole app was written completely only inside the main.cpp.
This is far from optimal behavior but I am a noob in app development.
Actually, this is the first time I've written something like this at all.

I had some help from ai to understand the whole color processing.
Yes, judge me.



# Why?
Well the options we had on Raspberry Pi (and it's linux based operating systems) actually caused a lot of input lag and weren't really fun to deal with.
For example mpv and vlc just aren't meant for this, since they are basically multimedia video players and that's why I felt there was a need for this application.
Also the upscaling that happened using applications like mpv and vlc weren't as sharp as I would have liked it.



# Features
## ~~1. Change internal render resolution~~
``` 
--res-720
--res-1080
--res-2160
```
~~These switches let you choose between 720p, 1080p and 2160p internal render resolution.\
The internal render resolution is the kind of resolution that happens from within the application itself.\
\
It does not mean that it actually outputs at that resolution. The output resolution is  depending on what the current output resolution of your Raspberry Pi is.
The Final output will be scaled to the current output resolution of your Raspberry Pi.\
\
**If nothing is provided** when executing this software, the default value is the current output resolution of your Raspberry Pi.~~
>[!CAUTION]
>Apparently, I removed this in one of my earlier builds and left some parts of it inside the main.cpp. The reason for this probably was that it does not have any effect on performance at all. This currently has no effect at all. The output completely depends on your current output resolution.
>Don't use ultrawide, it could have unforseen effects. Maybe this will be added back in again at some point, but there are no plans for now.

>[!NOTE]
>As of today, 26th of August 2026, I reimplemented this feature for testing in a personal build. It had to process the image twice. Once when upscaling to 720p, and then again to sort of stretch the image to the current screen size. This causes the GPU and CPU to have a lot of stuff to do. Even tho they weren't fully exhausted. Thus leading to more input lag and stuttering. That probably was the reason I removed it on build v1.6. For now we will leave this feature out. If you want to use this software, just make sure you use it on 16:9 output resolution. If someone much more versed in all this can tackle that, you would be more than welcome.

## 2. Modify RGB Color range BEFORE the conversion to RGB
```
--rgb-full
--rgb-limited
```
Check the folder "**example_screenshots**" to see what it does.

**If nothing is provided** when executing, the default mode is full.

## 3. Modify RGB Color range AFTER the conversion to RGB
```
--post-full
--post-limited
```
Check the folder "**example_screenshots**" to see what it does.

**If nothing is provided** when executing, the default mode is full.

## 4. Set the FPS limit
```
--fps-30
--fps-60
```
The udcd_uvc plugin has it's limits. If you want full native resolution of the PSVita (544p), you better stick with 30 fps, as it provides the most stutter free experience.

Check the [official Git repo for the udcd_uvc plugin](https://github.com/xerpi/vita-udcd-uvc) to have an overview of what the limits are. 

**If nothing is provided** when executing, the default mode is "30".

## 5. PSVita udcd_uvc output resolution
```
--vita-720
--vita-544
--vita-504
--vita-488
--vita-272
```

* 1280x720 @ 30 FPS 
	* is used by the sharpscale vita plugin
* 960x544 @ 30 FPS and (less than) 60 FPS
* 896x504 @ 30 FPS and (almost) 60 FPS
* 864x488 @ 30 FPS and 60 FPS
* 480x272 @ 30 FPS and 60 FPS

Check the [official Git repo for the udcd_uvc plugin](https://github.com/xerpi/vita-udcd-uvc) to have an overview of what the limits are. 

**If nothing is provided** when executing, the default mode is "544".

## 6. Set the output filter
```
--filter-bilinear
--filter-nearest
```

Sets the output filter to either nearest neighbour or bilinear.\
Bilinear can help to smooth the edges. Possibly good for some games that feel a bit rough with sharp pixels.\
\
You can also switch between both filter options by pressing F1 or F2 when the application is running.
* F1 = Nearest Neighbour
* F2 = Bilinear

**If nothing is provided** when executing, the default mode is "nearest".

## 7. Raspberry Pi 3B + Support
```
--pi3bp
```
This switch makes it possible for the software to run on a Pi 3B+. It changes the way things are handled under the hood. It forces MMAP instead of DMABUF. If you face any performance related issues, you might want to use that switch. You can also use that switch when executing the software on a Pi4 or Pi5.

**If nothing is provided** when executing, the default mode is "DMABUF", not the Pi 3B + supported "MMAP". So you have to actively provide that switch in order to make it run on a Pi 3B +.

## 8. Enable Audio over USB (v2.0+)
This only works, if you are using the [VitaUSBStream plugin](https://github.com/BMK-Studio/VitaUSBStream) on your PSVita and if you are on [v2.0 or later of the vita-uvc-app](https://github.com/EduardMaschine/vita-uvc-app/releases).
```
--audio
```
This switch enables the audio output coming in via USB over to HDMI by executing a pw-loopback command.

# Example execution
```
bash vita_uvc_app --fps-30 --vita-544 --res-2160 --filter-nearest --rgb-full --post-full --pi3bp 
```
This executes the software with mostly default values, except for the "--pi3bp" mode, but you get the idea. 

You can freely change the values or leave switches out so the default modes takes over.

# Tested Pi models
* Raspberry Pi 3B +
	- No issues so far
	- Tested on 720p and 1080p
* Raspberry Pi 4B
	- No issues so far
	- Performs just like on a Pi5
	- Tested on 4k (2160p)
* Raspberry Pi 5
	- No issues so far
	- Tested on 4k (2160p)

## What about the Pi Zero 2W?
If you have a Pi Zero 2W, please test this software in the --pi3bp mode. Unfortunately I cannot get my hands on that model as it's sold out for a very long time now. The current price tag is just far too high. With that price you can currently get a Pi5 2GB model... so I won't buy it and won't be able to test this myself. If any performance issues happen on that model, please feel free to fix it yourself if you are cappable. If you cannot, please wait until I can finally lay my hands on that model.

# How to compile it?
Use this:
```
g++ main.cpp -o vita_uvc_app -std=c++17 -lX11 -lEGL -lGLESv2
```
I work exclusively on the Pi itself. The goal was not to install anything special, so a fresh and new installation of the latest Raspbian OS (lite) can compile and run it.

# Known Issues
## 1. Slight frame loss (minimal stutter)
It happens from time to time, I cannot point my finger at it. Sometimes it's there, sometimes it's not at all. I suspect the PSVita's performance, but it could be my spaghetti code too.
## 2. Audio channels swapped
In version 2.0 I've added audio support via pw-loopback. I also implemented an ALSA solution as well during development, but ended up dropping it as it did not perform that well and also suffered from swapped audio channels.
I cannot for sure tell what's happening here, but there is a great chance that the left channel and right audio channels are swapped. This happens when the software starts, or when you reconnect the PSVita to the Pi.

This does NOT happen on OBS. So I guess it is somehow an issue in my code or how pipewire or alsa work.
If you want to help me out here, please feel free to, I wasted a lot of time fixing this with no success.

# What is the future of this application?
Actually I do not know. For me personally it is completed.
Maybe if I can get my hands on a Pi Zero 2W and it does not run well, I will update the app.
But for now that's it.
I simply don't have time to maintain it really. 
If you want to build up on the horrible code of this application, please feel free to do so.

# Personal recommendation
Use the following configs for the best output:
* --vita-544
* --rgb-full
* --post-full
* --fps-30
* --pi3bp
* --audio

Most of these options are default except for --pi3bp and --audio. 

So these are the only two you actually have to provide when launching the software.

We do 30fps here, because in it's current state, the UVC plugin just cannot provide steady full 60fps.

The pixels look mostly perfect and the overall quality is nice. And if you'd ask me, it even looks better than using OBS and upscaling it there with point or area scaling (nearest neighbour is not an option here).
Following an example of where this software just works better.

<img width="2311" height="806" alt="OBS-Screenshot 2026-08-26 00-53-15 - Kopie" src="https://github.com/user-attachments/assets/e812f749-d533-42f5-bd1a-e05069a0589e" />
On the left you see OBS, on the right you see vita-uvc-app running on a pi4b.

Both were upscaling to 2160p. The pixels and colors are just much more accurate on vita-uvc-app.

# Shoutouts to:
BenMitnicK, for the [VitaUSBStream plugin](https://github.com/BMK-Studio/VitaUSBStream) which expanded the udcd_uvc plugin by adding audio support via USB

xerpi, for the [udcd_uvc plugin](https://github.com/xerpi/vita-udcd-uvc)

SilentNightx, for the [vitadock plus](https://github.com/SilentNightx/VitaDockPlus) and the community around this for the great contact and the whole existence in general

# Screenshots
<img width="3840" height="2160" alt="Screenshot 2026-08-24 15-37-12" src="https://github.com/user-attachments/assets/648e06e8-a621-4087-a953-611d6327dcb3" />

<img width="3840" height="2160" alt="Screenshot 2026-08-24 15-40-05" src="https://github.com/user-attachments/assets/8d925fe4-04fa-4e40-822b-d6317ee54dd5" />

<img width="3840" height="2160" alt="Screenshot 2026-08-24 15-34-09" src="https://github.com/user-attachments/assets/3871a3b7-16c1-4c51-9cbc-4ab9f4e07fb9" />

<img width="3840" height="2160" alt="Screenshot 2026-08-24 15-44-33" src="https://github.com/user-attachments/assets/c7f49bbc-e997-4bae-811c-8404ccc924a5" />

<img width="3840" height="2160" alt="Screenshot 2026-08-24 15-45-09" src="https://github.com/user-attachments/assets/554bb9d3-c97d-40d8-8cf6-4cc0864ac463" />


# Have fun.
And know that you are valuable.
