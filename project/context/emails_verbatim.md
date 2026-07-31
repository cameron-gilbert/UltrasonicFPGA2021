# Email Chain — Verbatim

All emails as received/sent, chronological. Cameron = Pacific, Gazor = Eastern, Nikaein = Iran (UTC+3:30).

---

## B1 — Dr. Nikaein → all (reply to the plan email)
"I think starting with simulation and MATLAB development is a very good approach. It will allow us to evaluate the ideas and identify potential challenges before moving to hardware and real data. I am also currently working with Professor Gazor on completing a joint paper, which should be finished within the next week or two. After that, I expect to have more time available to focus on improving the hardware platform and supporting the next stages of the project. Best regards, Hossein"

## B2 — Gazor → Cameron (the pivotal off-the-shelf instruction; typos preserved, "mailman" = Kalman autocorrect)
"We would need 3 gyro, 3 accelerometer and 3gravity sensors. We need to measure the orientation and location of the device. It is not bad to have a gps. There may be out of shelve products which have everything already integrated. Note that integrating the output of these sensors needs to be processed in some extended mailman filters …. This is why if we find some out of shelve products, we will be able to spend our time on new stuff instead of developing those filtering processes to get accurate location/orientation info. In most current good cell phone such combinations (3 gyro, 3 accel….. plus algorithms ….) is used and that would be nice we could use something that exist in cellphone industry. Get input from dr Nikaein to see if he has any information about possible products."

## B4 — Dr. Nikaein → Gazor + Cameron (BNO085 recommendation)
"Dear Saeed and Cameron, I did a quick review of available IMU options, and one device that looks particularly attractive for this project is the BNO085. Unlike many standard IMUs, it includes onboard sensor fusion and can directly provide orientation information (e.g., quaternions, roll, pitch, and yaw), which could save us from implementing and tuning these algorithms ourselves. From a hardware perspective, it can communicate through I²C, SPI, or UART. The simplest approach would likely be to connect it to the existing FPGA/Zynq platform through I²C. Cameron, could you check whether there are any available pins or interfaces on the current board that could be used for such a connection? Saeed's suggestion of using a smartphone as an integrated sensing platform is also interesting. Modern phones already contain high-quality IMUs and sensor-fusion software. It may be worth investigating how easily motion and orientation data can be accessed and streamed to the laptop through Bluetooth or USB, and whether this could provide a practical alternative for the initial development stage. Best regards, Hossein"

## B7 — Dr. Nikaein → Cameron (J12 OK)
"Dear Cameron, That sounds good. Since the IMU can be connected independently through J12, you should be able to test it without affecting the current hardware design running on the board. Please keep me informed of the results of your tests and any issues you encounter. Best regards,"

## B9 — Gazor → Cameron (~Jul 13; receipts/ERS + second-device vision)
"Dear Cameron, Great, I had a lengthy discussion with him today—please keep your receipts, and once you have accumulated enough expenses, you can file a claim through our ERS system. I will then provide you with my account details for reimbursement. Also, please note that we will soon (after summer) set up a second enhanced device, possibly with more microphones. The goal is to enable two devices to collaborate in localizing and separating sound sources in 3D. Best regards,"

## B10 — Cameron → Gazor (~Jul 14)
"Dear Professor Gazor, Thank you for arranging the reimbursement — much appreciated. I'll keep track of my receipts. The second enhanced device sounds like a fantastic next step — I'm genuinely excited about that direction, and I'll keep it in mind as I work through the motion-compensation simulations this summer. Happy to dig into it further whenever the timing's right. Best regards, Cameron"

## Zack thread
Cameron → Zack:
"Hey Zach, Just reaching out again to see where you are at with your software. Not sure if you are in the same class as me but I just received an email indicating a final report is due on the 4th of august, which is quite soon. Cameron"

Zack → Cameron:
"Hi Cameron, Thank you for reaching out and for letting me know about the final report deadline. I currently have an emergency and am in Toronto right now. I understand that you are on a tight schedule, and I want to let you know that the hardware is available. I will be free on Wednesday or Thursday, and I can come back to deliver the hardware to you. Please let me know which time works better for you. Thanks again for checking in. Best, Zack Zhao"

Zack → Cameron (after Cameron's ship-instead / options message):
"Hi Cameron, Thank you so much for checking in. I can proceed with Option 1 later this afternoon. There is no need to provide a prepaid shipping label. Please send me your address, and I will ship the item and keep the receipt. I understand that you are working with a tight schedule, and I would also like to make this process as efficient as possible. Yours sincerely, Zack Zhao"

## Jul 19 — Cameron → Gazor + Nikaein (first results email; paraphrase of sent version)
Rotation-only sim results attached; setup (known yaw → corrupted sensor readings → counter-rotate synthetic acoustic frames → score vs truth); main result latency dominates (0.47° vs 0.41° ideal; 7.09° uncompensated); proposed next step (sparse per-source format, multiple sources, complementary filter fusing acoustic-frame estimate with IMU); asked image-level vs that; BNO085 on order + phone/MATLAB Mobile interim.

## Jul 19 — Gazor → Cameron (THE PIVOT, verbatim)
"Dear Cameron,
It is great to see the progress you have made so far.
Ultimately, our goal is to obtain accurate estimates of the device's x, y, and z position, velocity, acceleration, and orientation angles. This is why, in the next phase of the project, we will need to integrate multiple sensors. What you have completed so far provides a solid foundation.
Once we have access to all the raw measurements from the sensors, the next step will be to feed them into a tracking and sensor-fusion algorithm. Such algorithms process the various sensor measurements and produce significantly more accurate estimates of the device state than any individual sensor can provide. Fortunately, there are already well-established software tools and frameworks available for this purpose. Since the underlying algorithms are quite sophisticated and developing them from scratch would require substantial effort and expertise well beyond the scope of ELEC 497, I recommend focusing on identifying and leveraging existing solutions rather than attempting a new implementation.
We should schedule a meeting to discuss your plan in more detail. Since you will need to submit your ELEC 497 report sometime in August, my suggestion is to temporarily set aside the advanced tracking and fusion stage and concentrate on completing the sensor integration phase. I think this would be a more realistic and achievable objective for the project timeline.
I also recommend arranging a short meeting with Dr. Nikaein so that we can decide on the tentative material to be included in your report. If we can successfully integrate all of our sensors with the current data-collection platform, we will have a strong experimental foundation. The more advanced signal processing and MATLAB-based analysis can then follow afterward.
In other words, my recommendation is to prioritize completing the hardware integration and ensuring that all sensors are operational and collecting data reliably. Once that infrastructure is in place, the subsequent processing stages will be much easier to develop and evaluate.
Best regards,"

## Jul 20 — Gazor → Cameron (module order approval)
"Great. Once you confirm with dr Nikaein please go ahead and order the modules we need. Please keep your receipts and once all expenses paid you fill up a claim form on ERS so that u could reimburse you for these orders. Send an invite and we will meet soon, I prefer afternoons …"

## Jul 21 — Nikaein → Gazor + Cameron (module decision + availability)
"Dear Professor Gazor and Cameron,
Thank you, Cameron.
Regarding the GNSS module, the main advantage of the ZED-F9R is its built-in dead reckoning capability through integrated sensor fusion, whereas the basic u-blox module provides standard GNSS positioning at a much lower cost. Since we already have the BNO085 IMU and this is currently a laboratory-phase project, I think the basic u-blox receiver should be sufficient for our present needs. We can always upgrade to a more advanced module later if future experiments require it.
As for the barometric sensor, although our platform operates on a horizontal surface and we do not currently require accurate altitude estimation, the sensor is inexpensive. I think it is worthwhile to include it as well, since it may prove useful for future developments and experiments.
Regarding the meeting, I am usually available around 2:00 PM Ottawa time, or late evening Ottawa time. Either time works well for me.
Best regards, Hossein Nikaein"

## Jul 22 — Gazor → all (meeting timing)
"First times is fine with me. Just confirm. Monday I may be in the way and could attend while driving"

## Jul 22 — Gazor → Cameron (fragmented; interpreted)
"Let here from him if … worst case we can have separate meeting. I talked to him about …. Tonight … You discuss with him and … I mostly need to talk to you about paper works..."
[Interpretation: wait for Nikaein; meet all three if possible else separate; Cameron works the technical side (modules/plan) with Nikaein; Gazor's own meeting with Cameron is mostly paperwork/admin, not the technical plan.]

## Jul 23 — Nikaein → Gazor + Cameron (conference paper structure)
"I am sharing three papers that I found highly relevant to our project. Two of them are published in IEEE Transactions on Ultrasonics, Ferroelectrics, and Frequency Control (TUFFC), and all three follow a similar research direction.
After reading them, I noticed that their platforms are quite similar to ours in terms of the overall system architecture. Their main contribution is the development and experimental validation of a real-time ultrasonic phased-array platform, while the signal processing is mostly based on conventional beamforming techniques rather than introducing fundamentally new algorithms.
I believe these papers provide a good starting point for defining the scope and structure of our conference paper. Instead of reproducing a similar platform paper, we could leverage our 128-channel FPGA-based acquisition system and focus on its experimental capabilities, possibly complemented by a comparative evaluation of beamforming algorithms using real measured data.
As a preliminary idea, the conference paper could be organized as follows:
1. Introduction — Motivation and related work; Our contributions
2. System Architecture — 128-channel ultrasonic array; FPGA-based acquisition system; Synchronization and Ethernet communication
3. Signal Processing — Data preprocessing; Delay-and-Sum beamforming; (Optional) Additional beamforming algorithms
4. Experimental Setup
5. Experimental Results — Localization performance; Beam patterns; Processing latency; Comparison of beamforming methods
6. Conclusion and Future Work
I think these papers can serve as a useful guideline for the overall structure of our paper while allowing us to emphasize the unique aspects of our platform.
I would be interested to hear your thoughts on this possible direction for the ICASSP submission.
Best regards, Hossein"

## Jul 23 — Gazor → Cameron (short note)
"I dumped some papers in addition to those three to overleaf and set of the skeleton of the paper…
@Cameron Gilbert you can take a look at the papers ..,"

## Jul 26 — Cameron → Nikaein (SENT; reply on structure + FPGA source)
"Dear Dr. Nikaein,
Thanks — I've read the three papers and agree with the direction and structure. The Real-Time 3-D Imaging paper is the closest match to our platform and a good structural template.
Could you also share the FPGA source (HDL + Vivado project)? I only have the compiled platform and need it to fold the IMU/GNSS into the packet stream. Hardware arrives tomorrow after a shipping delay, so I'd like to start in parallel. Happy to meet over Teams.
Best regards, Cameron"

## Jul 26 — Cameron → Gazor (DRAFT; reply on papers/skeleton)
"Dear Professor Gazor,
Thanks — I've gone through the papers and the skeleton, and the direction makes sense. I've shared some thoughts with Dr. Nikaein and will start drafting in Overleaf.
Is there a particular paper or aspect you'd like us to focus on most closely as the model for our write-up?
The acoustic holography paper caught my interest in particular — a bit outside our imaging focus, but a direction I'd find interesting to explore later on.
Best regards, Cameron"
