<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.4.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="53" name="tGND_GNDA" color="7" fill="9" visible="no" active="no"/>
<layer number="54" name="bGND_GNDA" color="1" fill="9" visible="no" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="no" active="no"/>
<layer number="57" name="tCAD" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="7" fill="1" visible="yes" active="yes"/>
<layer number="100" name="Top CU 2" color="12" fill="1" visible="yes" active="yes"/>
<layer number="101" name="Gehäuse" color="7" fill="1" visible="no" active="no"/>
<layer number="102" name="Mittellin" color="7" fill="1" visible="no" active="no"/>
<layer number="103" name="tMap" color="7" fill="1" visible="yes" active="yes"/>
<layer number="104" name="Name" color="7" fill="1" visible="yes" active="yes"/>
<layer number="105" name="Beschreib" color="9" fill="1" visible="no" active="no"/>
<layer number="106" name="BGA-Top" color="4" fill="1" visible="no" active="no"/>
<layer number="107" name="BD-Top" color="5" fill="1" visible="no" active="no"/>
<layer number="108" name="fp8" color="7" fill="1" visible="yes" active="yes"/>
<layer number="109" name="fp9" color="7" fill="1" visible="yes" active="yes"/>
<layer number="110" name="fp0" color="7" fill="1" visible="yes" active="yes"/>
<layer number="111" name="LPC17xx" color="7" fill="1" visible="yes" active="yes"/>
<layer number="112" name="tSilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="113" name="IDFDebug" color="4" fill="1" visible="yes" active="yes"/>
<layer number="116" name="Patch_BOT" color="9" fill="4" visible="yes" active="yes"/>
<layer number="121" name="_tsilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="122" name="_bsilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="123" name="tTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="124" name="bTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="125" name="_tNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="126" name="_bNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="127" name="_tValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="128" name="_bValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="131" name="tAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="132" name="bAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="144" name="Drill_legend" color="7" fill="1" visible="yes" active="yes"/>
<layer number="150" name="Notes" color="7" fill="1" visible="yes" active="yes"/>
<layer number="151" name="HeatSink" color="14" fill="1" visible="yes" active="yes"/>
<layer number="152" name="_bDocu" color="7" fill="1" visible="yes" active="yes"/>
<layer number="153" name="FabDoc1" color="7" fill="1" visible="yes" active="yes"/>
<layer number="154" name="FabDoc2" color="7" fill="1" visible="yes" active="yes"/>
<layer number="155" name="FabDoc3" color="7" fill="1" visible="yes" active="yes"/>
<layer number="199" name="Contour" color="7" fill="1" visible="yes" active="yes"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="yes" active="yes"/>
<layer number="201" name="201bmp" color="2" fill="10" visible="yes" active="yes"/>
<layer number="202" name="202bmp" color="3" fill="10" visible="yes" active="yes"/>
<layer number="203" name="203bmp" color="4" fill="10" visible="yes" active="yes"/>
<layer number="204" name="204bmp" color="5" fill="10" visible="yes" active="yes"/>
<layer number="205" name="205bmp" color="6" fill="10" visible="yes" active="yes"/>
<layer number="206" name="206bmp" color="7" fill="10" visible="yes" active="yes"/>
<layer number="207" name="207bmp" color="8" fill="10" visible="yes" active="yes"/>
<layer number="208" name="208bmp" color="9" fill="10" visible="yes" active="yes"/>
<layer number="209" name="209bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="210" name="210bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="211" name="211bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="212" name="212bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="213" name="213bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="214" name="214bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="215" name="215bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="216" name="216bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="217" name="217bmp" color="18" fill="1" visible="no" active="no"/>
<layer number="218" name="218bmp" color="19" fill="1" visible="no" active="no"/>
<layer number="219" name="219bmp" color="20" fill="1" visible="no" active="no"/>
<layer number="220" name="220bmp" color="21" fill="1" visible="no" active="no"/>
<layer number="221" name="221bmp" color="22" fill="1" visible="no" active="no"/>
<layer number="222" name="222bmp" color="23" fill="1" visible="no" active="no"/>
<layer number="223" name="223bmp" color="24" fill="1" visible="no" active="no"/>
<layer number="224" name="224bmp" color="25" fill="1" visible="no" active="no"/>
<layer number="225" name="225bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="226" name="226bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="227" name="227bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="228" name="228bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="229" name="229bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="230" name="230bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="231" name="231bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="248" name="Housing" color="7" fill="1" visible="yes" active="yes"/>
<layer number="249" name="Edge" color="7" fill="1" visible="yes" active="yes"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="no"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="cooling" color="7" fill="1" visible="yes" active="yes"/>
<layer number="255" name="tmp" color="7" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="filip">
<description>Filip's custom library</description>
<packages>
<package name="CRYSTAL-SMD-7X5">
<wire x1="1.402" y1="-0.4094" x2="1.402" y2="-0.6094" width="0.127" layer="21"/>
<wire x1="-0.889" y1="1.27" x2="-0.889" y2="2.921" width="0.2032" layer="21"/>
<wire x1="5.969" y1="1.27" x2="5.969" y2="2.921" width="0.2032" layer="21"/>
<wire x1="3.937" y1="5.207" x2="1.143" y2="5.207" width="0.2032" layer="21"/>
<wire x1="3.937" y1="-1.016" x2="1.143" y2="-1.016" width="0.2032" layer="21"/>
<circle x="1.402" y="-0.5094" radius="0.3" width="0.127" layer="21"/>
<circle x="1.402" y="-0.5094" radius="0.2236" width="0.127" layer="21"/>
<circle x="1.402" y="-0.5094" radius="0.1" width="0.127" layer="21"/>
<smd name="P$1" x="0" y="0" dx="2" dy="1.8" layer="1" rot="R90"/>
<smd name="P$2" x="5.08" y="0" dx="2" dy="1.8" layer="1" rot="R90"/>
<smd name="P$3" x="5.08" y="4.2" dx="2" dy="1.8" layer="1" rot="R90"/>
<smd name="P$4" x="0" y="4.2" dx="2" dy="1.8" layer="1" rot="R90"/>
<text x="-1.016" y="5.842" size="1.27" layer="25" ratio="15">&gt;Name</text>
<text x="-1.016" y="-2.921" size="1.27" layer="27" ratio="15">&gt;Value</text>
</package>
<package name="CRYSTAL-SMD-7.5X5.2-6PIN">
<description>FXO-HC73 Series Oscillator. 7.5x5.2mm 6-pin package</description>
<wire x1="3.75" y1="2.6" x2="-3.75" y2="2.6" width="0.127" layer="51"/>
<wire x1="-3.75" y1="2.6" x2="-3.75" y2="-2.6" width="0.127" layer="51"/>
<wire x1="-3.75" y1="-2.6" x2="3.75" y2="-2.6" width="0.127" layer="51"/>
<wire x1="3.75" y1="-2.6" x2="3.75" y2="2.6" width="0.127" layer="51"/>
<wire x1="-3.8" y1="2.4" x2="-3.8" y2="-2.4" width="0.127" layer="21"/>
<wire x1="3.8" y1="2.4" x2="3.8" y2="-2.4" width="0.127" layer="21"/>
<circle x="-3" y="-0.5" radius="0.3605" width="0.127" layer="21"/>
<smd name="2" x="2.54" y="-2.1" dx="2" dy="1.8" layer="1" rot="R90"/>
<smd name="3" x="2.54" y="2.1" dx="2" dy="1.8" layer="1" rot="R90"/>
<smd name="4" x="-2.54" y="2.1" dx="2" dy="1.8" layer="1" rot="R90"/>
<smd name="1" x="-2.54" y="-2.1" dx="2" dy="1.8" layer="1" rot="R90"/>
<smd name="NC0" x="0" y="-2.1" dx="2" dy="1.4" layer="1" rot="R90"/>
<smd name="NC1" x="0" y="2.1" dx="2" dy="1.4" layer="1" rot="R90"/>
<text x="-3.6" y="3.4" size="0.4064" layer="25">&gt;NAME</text>
<text x="-3.7" y="-3.8" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="CRYSTAL-SMD-5X3">
<wire x1="-0.6" y1="1.6" x2="0.6" y2="1.6" width="0.2032" layer="21"/>
<wire x1="2.5" y1="0.3" x2="2.5" y2="-0.3" width="0.2032" layer="21"/>
<wire x1="0.6" y1="-1.6" x2="-0.6" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="0.3" x2="-2.5" y2="-0.3" width="0.2032" layer="21"/>
<smd name="1" x="-1.85" y="-1.15" dx="1.9" dy="1.1" layer="1"/>
<smd name="3" x="1.85" y="1.15" dx="1.9" dy="1.1" layer="1"/>
<smd name="4" x="-1.85" y="1.15" dx="1.9" dy="1.1" layer="1"/>
<smd name="2" x="1.85" y="-1.15" dx="1.9" dy="1.1" layer="1"/>
<text x="-2.54" y="1.905" size="0.4064" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="CRYSTAL-OSC-SMD-5X3">
<wire x1="-0.6" y1="1.6" x2="0.6" y2="1.6" width="0.2032" layer="21"/>
<wire x1="2.5" y1="0.3" x2="2.5" y2="-0.3" width="0.2032" layer="21"/>
<wire x1="0.6" y1="-1.6" x2="-0.6" y2="-1.6" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="0.3" x2="-2.5" y2="-0.3" width="0.2032" layer="21"/>
<circle x="-3.4986" y="-1.2794" radius="0.2653" width="0.127" layer="21"/>
<circle x="-3.4986" y="-1.2794" radius="0.1456" width="0.127" layer="21"/>
<circle x="-3.4986" y="-1.2794" radius="0.0284" width="0.127" layer="21"/>
<smd name="1" x="-1.85" y="-1.15" dx="1.9" dy="1.1" layer="1"/>
<smd name="3" x="1.85" y="1.15" dx="1.9" dy="1.1" layer="1"/>
<smd name="4" x="-1.85" y="1.15" dx="1.9" dy="1.1" layer="1"/>
<smd name="2" x="1.85" y="-1.15" dx="1.9" dy="1.1" layer="1"/>
<text x="-2.54" y="1.905" size="0.4064" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="0.4064" layer="27">&gt;VALUE</text>
</package>
<package name="SPH0641LU4H-1">
<description>Knowles MEMS microphone SPH0641LU4H-1

.023in microphone drill hole</description>
<wire x1="1.325" y1="2.46" x2="-1.325" y2="2.46" width="0.254" layer="51"/>
<wire x1="-1.325" y1="2.46" x2="-1.325" y2="-1.04" width="0.254" layer="51"/>
<wire x1="-1.325" y1="-1.04" x2="1.325" y2="-1.04" width="0.254" layer="51"/>
<wire x1="1.325" y1="-1.04" x2="1.325" y2="2.46" width="0.254" layer="51"/>
<wire x1="-1.365" y1="0" x2="-1.365" y2="-1.175" width="0.254" layer="21"/>
<wire x1="-1.365" y1="-1.175" x2="-0.465" y2="-1.175" width="0.254" layer="21"/>
<wire x1="-1.365" y1="1.675" x2="-1.365" y2="2.625" width="0.254" layer="21"/>
<wire x1="-1.365" y1="2.625" x2="1.365" y2="2.625" width="0.254" layer="21"/>
<wire x1="1.365" y1="2.625" x2="1.365" y2="1.65" width="0.254" layer="21"/>
<wire x1="0.465" y1="-1.175" x2="1.365" y2="-1.175" width="0.254" layer="21"/>
<wire x1="1.365" y1="-1.175" x2="1.365" y2="0" width="0.254" layer="21"/>
<wire x1="-0.16" y1="0.655" x2="-0.65" y2="0.16" width="0.3" layer="31" curve="57.004037" cap="flat"/>
<wire x1="0.655" y1="0.135" x2="0.21" y2="0.625" width="0.3" layer="31" curve="57.004037" cap="flat"/>
<wire x1="-0.655" y1="-0.16" x2="-0.16" y2="-0.65" width="0.3" layer="31" curve="57.004037" cap="flat"/>
<wire x1="0.16" y1="-0.655" x2="0.65" y2="-0.16" width="0.3" layer="31" curve="57.004037" cap="flat"/>
<circle x="0" y="0" radius="0.6625" width="0.3" layer="1"/>
<circle x="0" y="0" radius="0.6625" width="0.3" layer="29"/>
<smd name="4" x="0.8375" y="1.252" dx="0.725" dy="0.522" layer="1"/>
<smd name="2" x="-0.8375" y="1.252" dx="0.725" dy="0.522" layer="1"/>
<smd name="1" x="-0.8375" y="2.074" dx="0.725" dy="0.522" layer="1"/>
<smd name="5" x="0.8375" y="2.074" dx="0.725" dy="0.522" layer="1"/>
<smd name="3" x="0" y="-1.016" dx="0.508" dy="0.508" layer="1" stop="no" thermals="no" cream="no"/>
<text x="-1.959" y="2.863" size="0.762" layer="25">&gt;NAME</text>
<text x="-1.984" y="-2.492" size="0.7112" layer="27">&gt;VALUE</text>
<hole x="0" y="0" drill="0.6"/>
</package>
<package name="TE-A-3216">
<description>&lt;b&gt;SMD TANTALUM&lt;/b&gt;&lt;p&gt; 
EIA Code 3216, Size A, 3.2 mm x 1.6 mm, grid 0.0125 inch</description>
<wire x1="-1.2925" y1="1.0225" x2="0.405" y2="1.0225" width="0.1778" layer="21"/>
<wire x1="0.405" y1="1.0225" x2="1.2925" y2="1.0225" width="0.1778" layer="21"/>
<wire x1="-1.2925" y1="-1.0225" x2="0.405" y2="-1.0225" width="0.1778" layer="21"/>
<wire x1="0.405" y1="-1.0225" x2="1.2925" y2="-1.0225" width="0.1778" layer="21"/>
<wire x1="0.405" y1="1.0225" x2="0.405" y2="-1.0225" width="0.1778" layer="21"/>
<wire x1="-1.2925" y1="1.0225" x2="-1.2925" y2="-1.0225" width="0.1778" layer="51"/>
<wire x1="1.2925" y1="1.0225" x2="1.2925" y2="-1.0225" width="0.1778" layer="51"/>
<smd name="-" x="-1.27" y="0" dx="1.778" dy="1.143" layer="1" rot="R90"/>
<smd name="+" x="1.27" y="0" dx="1.778" dy="1.143" layer="1" rot="R90"/>
<text x="-1.5875" y="-2.2225" size="1.016" layer="21" ratio="12">&gt;VALUE</text>
<text x="-1.5875" y="1.27" size="1.016" layer="21" ratio="12">&gt;NAME</text>
<rectangle x1="-2.25" y1="-0.15" x2="-0.65" y2="0.15" layer="51" rot="R90"/>
<rectangle x1="0.3" y1="-1" x2="1.25" y2="0.95" layer="51"/>
<rectangle x1="1.0963" y1="0.2963" x2="1.7963" y2="0.6037" layer="51" rot="R90"/>
<rectangle x1="1.0963" y1="-0.6037" x2="1.7963" y2="-0.2963" layer="51" rot="R90"/>
</package>
<package name="TE-B-3528">
<description>&lt;b&gt;SMD TANTALUM&lt;/b&gt;&lt;p&gt; 
EIA Code 3528, Size B, 3.5 mm x 2.8 mm, grid 0.0125 inch</description>
<wire x1="-1.7925" y1="1.5225" x2="0.805" y2="1.5225" width="0.1778" layer="21"/>
<wire x1="0.805" y1="1.5225" x2="1.7925" y2="1.5225" width="0.1778" layer="21"/>
<wire x1="-1.7925" y1="-1.5225" x2="0.805" y2="-1.5225" width="0.1778" layer="21"/>
<wire x1="0.805" y1="-1.5225" x2="1.7925" y2="-1.5225" width="0.1778" layer="21"/>
<wire x1="-0.1562" y1="0.9287" x2="0.4788" y2="0.9287" width="0.1778" layer="21"/>
<wire x1="0.1613" y1="1.2462" x2="0.1613" y2="0.6112" width="0.1778" layer="21"/>
<wire x1="0.805" y1="1.5225" x2="0.805" y2="-1.5225" width="0.1778" layer="21"/>
<wire x1="-1.7925" y1="1.5225" x2="-1.7925" y2="-1.5225" width="0.1778" layer="51"/>
<wire x1="1.7925" y1="1.5225" x2="1.7925" y2="-1.5225" width="0.1778" layer="51"/>
<smd name="-" x="-1.905" y="0" dx="2.54" dy="1.524" layer="1" rot="R90"/>
<smd name="+" x="1.905" y="0" dx="2.54" dy="1.524" layer="1" rot="R90"/>
<text x="-2.2225" y="-2.8575" size="1.016" layer="21" ratio="12">&gt;VALUE</text>
<text x="-2.2225" y="1.905" size="1.016" layer="21" ratio="12">&gt;NAME</text>
<rectangle x1="-3.28" y1="-0.18" x2="-0.68" y2="0.18" layer="51" rot="R90"/>
<rectangle x1="0.8" y1="-1.5" x2="1.8" y2="1.45" layer="51"/>
<rectangle x1="1.4463" y1="0.5463" x2="2.4463" y2="0.8537" layer="51" rot="R90"/>
<rectangle x1="1.4463" y1="-0.9537" x2="2.4463" y2="-0.6463" layer="51" rot="R90"/>
</package>
<package name="TE-C-6032">
<description>&lt;b&gt;SMD TANTALUM&lt;/b&gt;&lt;p&gt; 
EIA Code 6032, Size C, 6.0 mm x 3.2 mm, grid 0.0125 inch</description>
<wire x1="-2.9925" y1="1.7225" x2="1.205" y2="1.7225" width="0.1778" layer="21"/>
<wire x1="1.205" y1="1.7225" x2="2.9925" y2="1.7225" width="0.1778" layer="21"/>
<wire x1="-2.9925" y1="-1.7225" x2="1.205" y2="-1.7225" width="0.1778" layer="21"/>
<wire x1="1.205" y1="-1.7225" x2="2.9925" y2="-1.7225" width="0.1778" layer="21"/>
<wire x1="0.1938" y1="0.9287" x2="0.8288" y2="0.9287" width="0.1778" layer="21"/>
<wire x1="0.5113" y1="1.2462" x2="0.5113" y2="0.6112" width="0.1778" layer="21"/>
<wire x1="1.205" y1="1.7225" x2="1.205" y2="-1.7225" width="0.1778" layer="21"/>
<wire x1="-2.9925" y1="1.7225" x2="-2.9925" y2="-1.7225" width="0.1778" layer="51"/>
<wire x1="2.9925" y1="1.7225" x2="2.9925" y2="-1.7225" width="0.1778" layer="51"/>
<smd name="-" x="-2.8575" y="0" dx="2.794" dy="2.54" layer="1" rot="R90"/>
<smd name="+" x="2.8575" y="0" dx="2.794" dy="2.54" layer="1" rot="R90"/>
<text x="-2.9925" y="-2.9925" size="1.016" layer="21" ratio="12">&gt;VALUE</text>
<text x="-2.9925" y="2.29" size="1.016" layer="21" ratio="12">&gt;NAME</text>
<rectangle x1="2.4963" y1="0.7463" x2="3.7463" y2="1.0038" layer="51" rot="R90"/>
<rectangle x1="-4.655" y1="-0.155" x2="-1.655" y2="0.155" layer="51" rot="R90"/>
<rectangle x1="1.3" y1="-1.75" x2="2.9925" y2="1.6" layer="51"/>
<rectangle x1="2.53" y1="-1.0376" x2="3.7125" y2="-0.7801" layer="51" rot="R90"/>
</package>
<package name="TE-D-7343">
<description>&lt;b&gt;SMD TANTALUM&lt;/b&gt;&lt;p&gt; 
EIA Code 7343, Size D, 7.3 mm x 4.3 mm, grid 0.0125 inch</description>
<wire x1="-3.3925" y1="2.2225" x2="1.205" y2="2.2225" width="0.1778" layer="21"/>
<wire x1="1.205" y1="2.2225" x2="3.3925" y2="2.2225" width="0.1778" layer="21"/>
<wire x1="-3.3925" y1="-2.2225" x2="1.205" y2="-2.2225" width="0.1778" layer="21"/>
<wire x1="1.205" y1="-2.2225" x2="3.3925" y2="-2.2225" width="0.1778" layer="21"/>
<wire x1="0.1763" y1="1.4287" x2="0.8113" y2="1.4287" width="0.1778" layer="21"/>
<wire x1="0.4938" y1="1.7462" x2="0.4938" y2="1.1112" width="0.1778" layer="21"/>
<wire x1="1.205" y1="2.2225" x2="1.205" y2="-2.2225" width="0.1778" layer="21"/>
<wire x1="-3.3925" y1="2.2225" x2="-3.3925" y2="-2.2225" width="0.1778" layer="51"/>
<wire x1="3.3925" y1="2.2225" x2="3.3925" y2="-2.2225" width="0.1778" layer="51"/>
<smd name="-" x="-3.175" y="0" dx="3.81" dy="3.175" layer="1" rot="R90"/>
<smd name="+" x="3.175" y="0" dx="3.81" dy="3.175" layer="1" rot="R90"/>
<text x="-3.4925" y="-3.4925" size="1.016" layer="21" ratio="12">&gt;VALUE</text>
<text x="-3.4925" y="2.54" size="1.016" layer="21" ratio="12">&gt;NAME</text>
<rectangle x1="2.7576" y1="0.9525" x2="4.3451" y2="1.27" layer="51" rot="R90"/>
<rectangle x1="-5.4563" y1="-0.1588" x2="-1.6463" y2="0.1587" layer="51" rot="R90"/>
<rectangle x1="1.3" y1="-2.2225" x2="3.3925" y2="2.2" layer="51"/>
<rectangle x1="2.7575" y1="-1.2701" x2="4.345" y2="-0.9526" layer="51" rot="R90"/>
</package>
<package name="F60">
<description>&lt;b&gt;SMD POLCAP&lt;/b&gt;&lt;p&gt;
SMD (Chip) Vertical United Chem Con F60</description>
<wire x1="2.4" y1="-3.3" x2="-3.3" y2="-3.3" width="0.2032" layer="21"/>
<wire x1="-3.3" y1="-3.3" x2="-3.3" y2="-1.1" width="0.2032" layer="21"/>
<wire x1="-3.3" y1="-1.1" x2="-3.3" y2="1.1" width="0.2032" layer="51"/>
<wire x1="-3.3" y1="1.1" x2="-3.3" y2="3.3" width="0.2032" layer="21"/>
<wire x1="-3.3" y1="3.3" x2="2.4" y2="3.3" width="0.2032" layer="21"/>
<wire x1="3.3" y1="2.4" x2="3.3" y2="1.1" width="0.2032" layer="21"/>
<wire x1="3.3" y1="1.1" x2="3.3" y2="-1.1" width="0.2032" layer="51"/>
<wire x1="3.3" y1="-1.1" x2="3.3" y2="-2.4" width="0.2032" layer="21"/>
<wire x1="3.3" y1="-2.4" x2="2.4" y2="-3.3" width="0.2032" layer="21"/>
<wire x1="2.4" y1="3.3" x2="3.3" y2="2.4" width="0.2032" layer="21"/>
<wire x1="-2.95" y1="-0.9" x2="2.95" y2="-0.95" width="0.2032" layer="21" curve="145.181395" cap="flat"/>
<wire x1="-2.95" y1="-0.9" x2="-2.95" y2="0.95" width="0.2032" layer="51" curve="-34.818605" cap="flat"/>
<wire x1="-2.95" y1="0.95" x2="2.95" y2="0.9" width="0.2032" layer="21" curve="-145.181395" cap="flat"/>
<wire x1="2.95" y1="-0.95" x2="2.95" y2="0.9" width="0.2032" layer="51" curve="34.818605" cap="flat"/>
<smd name="+" x="2.79" y="0" dx="3.5" dy="1.6" layer="1"/>
<smd name="-" x="-2.79" y="0" dx="3.5" dy="1.6" layer="1"/>
<text x="-3.175" y="3.81" size="1.016" layer="25" ratio="12">&gt;NAME</text>
<text x="-3.175" y="-4.445" size="0.8128" layer="27" ratio="12">&gt;VALUE</text>
</package>
<package name="H70_OR_HA0">
<description>&lt;b&gt;SMD POLCAP&lt;/b&gt;&lt;p&gt;
SMD (Chip) Vertical United Chem Con H70 or HA0 size</description>
<wire x1="3.3" y1="-4.2" x2="-4.2" y2="-4.2" width="0.2032" layer="21"/>
<wire x1="-4.2" y1="-4.2" x2="-4.2" y2="-1.1" width="0.2032" layer="21"/>
<wire x1="-4.2" y1="-1.1" x2="-4.2" y2="1.1" width="0.2032" layer="51"/>
<wire x1="-4.2" y1="1.1" x2="-4.2" y2="4.2" width="0.2032" layer="21"/>
<wire x1="-4.2" y1="4.2" x2="3.3" y2="4.2" width="0.2032" layer="21"/>
<wire x1="4.2" y1="3.3" x2="4.2" y2="1.1" width="0.2032" layer="21"/>
<wire x1="4.2" y1="1.1" x2="4.2" y2="-1.1" width="0.2032" layer="51"/>
<wire x1="4.2" y1="-1.1" x2="4.2" y2="-3.3" width="0.2032" layer="21"/>
<wire x1="4.2" y1="-3.3" x2="3.3" y2="-4.2" width="0.2032" layer="21"/>
<wire x1="3.3" y1="4.2" x2="4.2" y2="3.3" width="0.2032" layer="21"/>
<wire x1="-3.75" y1="-1.05" x2="3.75" y2="-1.05" width="0.2032" layer="21" curve="148.008335" cap="flat"/>
<wire x1="-3.75" y1="-1.05" x2="-3.75" y2="1.05" width="0.2032" layer="51" curve="-31.284493" cap="flat"/>
<wire x1="-3.75" y1="1.05" x2="3.75" y2="1.05" width="0.2032" layer="21" curve="-148.008335" cap="flat"/>
<wire x1="3.75" y1="1.05" x2="3.75" y2="-1.05" width="0.2032" layer="51" curve="-31.284493" cap="flat"/>
<smd name="+" x="3.675" y="0" dx="4.2" dy="2.2" layer="1"/>
<smd name="-" x="-3.5994" y="0" dx="4.2" dy="2.2" layer="1"/>
<text x="-4.1275" y="4.7625" size="1.016" layer="25" ratio="12">&gt;NAME</text>
<text x="-4.1275" y="-5.3975" size="0.8128" layer="27" ratio="12">&gt;VALUE</text>
</package>
<package name="E60">
<description>&lt;b&gt;SMD POLCAP&lt;/b&gt;&lt;p&gt;
SMD (Chip) Vertical United Chem Con E60&lt;p&gt;
5x5.7</description>
<wire x1="1.75" y1="-2.65" x2="-2.65" y2="-2.65" width="0.2032" layer="21"/>
<wire x1="-2.65" y1="-2.65" x2="-2.65" y2="-1.1" width="0.2032" layer="21"/>
<wire x1="-2.65" y1="-1.1" x2="-2.65" y2="1.1" width="0.2032" layer="51"/>
<wire x1="-2.65" y1="1.1" x2="-2.65" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-2.65" y1="2.65" x2="1.75" y2="2.65" width="0.2032" layer="21"/>
<wire x1="2.65" y1="1.75" x2="2.65" y2="1.1" width="0.2032" layer="21"/>
<wire x1="2.65" y1="1.1" x2="2.65" y2="-1.1" width="0.2032" layer="51"/>
<wire x1="2.65" y1="-1.1" x2="2.65" y2="-1.75" width="0.2032" layer="21"/>
<wire x1="2.65" y1="-1.75" x2="1.75" y2="-2.65" width="0.2032" layer="21"/>
<wire x1="1.75" y1="2.65" x2="2.65" y2="1.75" width="0.2032" layer="21"/>
<wire x1="-2.2" y1="-0.95" x2="2.2" y2="-0.95" width="0.2032" layer="21" curve="133.28887" cap="flat"/>
<wire x1="-2.2" y1="-0.95" x2="-2.2" y2="0.95" width="0.2032" layer="51" curve="-46.71113" cap="flat"/>
<wire x1="-2.2" y1="0.95" x2="2.2" y2="0.95" width="0.2032" layer="21" curve="-133.28887" cap="flat"/>
<wire x1="2.2" y1="-0.95" x2="2.2" y2="0.95" width="0.2032" layer="51" curve="46.71113" cap="flat"/>
<smd name="+" x="2.2225" y="0" dx="3" dy="1.6" layer="1"/>
<smd name="-" x="-2.2225" y="0" dx="3" dy="1.6" layer="1"/>
<text x="-2.54" y="3.175" size="1.016" layer="25" ratio="12">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="0.8128" layer="27" ratio="12">&gt;VALUE</text>
</package>
<package name="0805">
<description>&lt;b&gt;SMD CHIP CAP Polarized&lt;/b&gt;&lt;p&gt;
0805, grid 0.0125 inch</description>
<wire x1="-0.381" y1="0.66" x2="0.381" y2="0.66" width="0.1016" layer="51"/>
<wire x1="-0.356" y1="-0.66" x2="0.381" y2="-0.66" width="0.1016" layer="51"/>
<smd name="+" x="-0.9525" y="0" dx="1.27" dy="1.524" layer="1"/>
<smd name="-" x="0.9525" y="0" dx="1.27" dy="1.524" layer="1"/>
<text x="-0.9525" y="0.9525" size="1.016" layer="25" ratio="14">&gt;NAME</text>
<text x="-0.9525" y="-1.905" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.0916" y1="-0.7239" x2="-0.3415" y2="0.7262" layer="51"/>
<rectangle x1="0.3556" y1="-0.7239" x2="1.1057" y2="0.7262" layer="51"/>
<rectangle x1="-0.1588" y1="-0.4763" x2="0.1588" y2="0.4763" layer="35"/>
<wire x1="-1.93" y1="-1.016" x2="-1.93" y2="1.016" width="0.254" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="OSCILLATOR">
<wire x1="-7.62" y1="-5.08" x2="-7.62" y2="5.08" width="0.254" layer="94"/>
<wire x1="-7.62" y1="5.08" x2="7.62" y2="5.08" width="0.254" layer="94"/>
<wire x1="7.62" y1="5.08" x2="7.62" y2="-5.08" width="0.254" layer="94"/>
<wire x1="7.62" y1="-5.08" x2="-7.62" y2="-5.08" width="0.254" layer="94"/>
<text x="-7.62" y="5.842" size="1.27" layer="95">&gt;Name</text>
<text x="-7.62" y="-7.366" size="1.27" layer="96">&gt;Value</text>
<pin name="EN" x="-12.7" y="-2.54" visible="pin" length="middle"/>
<pin name="GND" x="12.7" y="-2.54" visible="pin" length="middle" rot="R180"/>
<pin name="OUT" x="12.7" y="2.54" visible="pin" length="middle" rot="R180"/>
<pin name="VCC" x="-12.7" y="2.54" visible="pin" length="middle"/>
</symbol>
<symbol name="SPH0641LU4H-1">
<description>Knowles MEMS microphone</description>
<wire x1="-7.62" y1="10.16" x2="-7.62" y2="-10.16" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-10.16" x2="7.62" y2="-10.16" width="0.254" layer="94"/>
<wire x1="7.62" y1="-10.16" x2="7.62" y2="10.16" width="0.254" layer="94"/>
<wire x1="7.62" y1="10.16" x2="-7.62" y2="10.16" width="0.254" layer="94"/>
<text x="-7.62" y="10.668" size="1.778" layer="95">&gt;NAME</text>
<text x="-7.62" y="-12.192" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VDD" x="-10.16" y="7.62" visible="pin" length="short"/>
<pin name="DATA" x="10.16" y="0" visible="pin" length="short" rot="R180"/>
<pin name="SELECT" x="10.16" y="5.08" visible="pin" length="short" rot="R180"/>
<pin name="CLK" x="10.16" y="-5.08" visible="pin" length="short" rot="R180"/>
<pin name="GND" x="-10.16" y="-5.08" visible="pin" length="short"/>
</symbol>
<symbol name="CPOL">
<wire x1="-2.54" y1="5.08" x2="-0.508" y2="5.08" width="0.1524" layer="94"/>
<wire x1="-0.508" y1="6.604" x2="-0.508" y2="5.08" width="0.254" layer="94"/>
<wire x1="-0.508" y1="5.08" x2="-0.508" y2="3.556" width="0.254" layer="94"/>
<wire x1="0.635" y1="6.604" x2="0.635" y2="3.556" width="0.254" layer="94"/>
<wire x1="0.635" y1="3.556" x2="1.397" y2="3.556" width="0.254" layer="94"/>
<wire x1="1.397" y1="6.604" x2="0.635" y2="6.604" width="0.254" layer="94"/>
<wire x1="1.397" y1="5.08" x2="5.08" y2="5.08" width="0.1524" layer="94"/>
<wire x1="1.397" y1="6.604" x2="1.397" y2="5.08" width="0.254" layer="94"/>
<wire x1="1.397" y1="5.08" x2="1.397" y2="3.556" width="0.254" layer="94"/>
<wire x1="-0.762" y1="6.604" x2="-0.508" y2="6.604" width="0.254" layer="94"/>
<wire x1="-0.508" y1="6.604" x2="-0.254" y2="6.604" width="0.254" layer="94"/>
<wire x1="-0.254" y1="6.604" x2="-0.254" y2="3.556" width="0.254" layer="94"/>
<wire x1="-0.254" y1="3.556" x2="-0.508" y2="3.556" width="0.254" layer="94"/>
<wire x1="-0.508" y1="3.556" x2="-0.762" y2="3.556" width="0.254" layer="94"/>
<wire x1="-0.762" y1="3.556" x2="-0.762" y2="6.604" width="0.254" layer="94"/>
<wire x1="2.5654" y1="6.8834" x2="2.5654" y2="5.9436" width="0.1524" layer="94"/>
<wire x1="2.1082" y1="6.4262" x2="3.048" y2="6.4262" width="0.1524" layer="94"/>
<text x="3.683" y="5.5626" size="1.778" layer="95">&gt;NAME</text>
<text x="-4.2545" y="1.1176" size="1.778" layer="96">&gt;VALUE</text>
<pin name="-" x="-2.54" y="5.08" visible="off" length="point" direction="pas"/>
<pin name="+" x="5.08" y="5.08" visible="off" length="point" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="OSCILLATOR" prefix="U">
<description>Generic 5x3 and 7x5 oscillators</description>
<gates>
<gate name="G$1" symbol="OSCILLATOR" x="0" y="0"/>
</gates>
<devices>
<device name="SMD" package="CRYSTAL-SMD-7X5">
<connects>
<connect gate="G$1" pin="EN" pad="P$1"/>
<connect gate="G$1" pin="GND" pad="P$2"/>
<connect gate="G$1" pin="OUT" pad="P$3"/>
<connect gate="G$1" pin="VCC" pad="P$4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD6" package="CRYSTAL-SMD-7.5X5.2-6PIN">
<connects>
<connect gate="G$1" pin="EN" pad="1"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="OUT" pad="3"/>
<connect gate="G$1" pin="VCC" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5X3" package="CRYSTAL-SMD-5X3">
<connects>
<connect gate="G$1" pin="EN" pad="1"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="OUT" pad="3"/>
<connect gate="G$1" pin="VCC" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5X3-PIN1-INDICATED" package="CRYSTAL-OSC-SMD-5X3">
<connects>
<connect gate="G$1" pin="EN" pad="1"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="OUT" pad="3"/>
<connect gate="G$1" pin="VCC" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="XTAL-11225" package="CRYSTAL-SMD-7X5">
<connects>
<connect gate="G$1" pin="EN" pad="P$1"/>
<connect gate="G$1" pin="GND" pad="P$2"/>
<connect gate="G$1" pin="OUT" pad="P$3"/>
<connect gate="G$1" pin="VCC" pad="P$4"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="XTAL-11225"/>
<attribute name="VALUE" value="16MHz"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="SPH0641LU4H-1">
<description>Knowles MEMS microphone - ultrasonic range</description>
<gates>
<gate name="G$1" symbol="SPH0641LU4H-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SPH0641LU4H-1">
<connects>
<connect gate="G$1" pin="CLK" pad="4"/>
<connect gate="G$1" pin="DATA" pad="1"/>
<connect gate="G$1" pin="GND" pad="3"/>
<connect gate="G$1" pin="SELECT" pad="2"/>
<connect gate="G$1" pin="VDD" pad="5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CPOL" prefix="C" uservalue="yes">
<description>polarized SMD caps&lt;p&gt;
Size A 3.2x1.6,
Size B 3.5x2.8,
Size C 6.0x3.2,
Size D (or N) 7.3x4.3 
0805&lt;p&gt;

United Chem   E60 5x5.7,  F60 6.3x5.7, H70 8x6.7  or HA0</description>
<gates>
<gate name="G$1" symbol="CPOL" x="0" y="-2.54"/>
</gates>
<devices>
<device name="A" package="TE-A-3216">
<connects>
<connect gate="G$1" pin="+" pad="+"/>
<connect gate="G$1" pin="-" pad="-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="B" package="TE-B-3528">
<connects>
<connect gate="G$1" pin="+" pad="+"/>
<connect gate="G$1" pin="-" pad="-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="C" package="TE-C-6032">
<connects>
<connect gate="G$1" pin="+" pad="+"/>
<connect gate="G$1" pin="-" pad="-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="D" package="TE-D-7343">
<connects>
<connect gate="G$1" pin="+" pad="+"/>
<connect gate="G$1" pin="-" pad="-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="F60" package="F60">
<connects>
<connect gate="G$1" pin="+" pad="+"/>
<connect gate="G$1" pin="-" pad="-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="H70" package="H70_OR_HA0">
<connects>
<connect gate="G$1" pin="+" pad="+"/>
<connect gate="G$1" pin="-" pad="-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="E60" package="E60">
<connects>
<connect gate="G$1" pin="+" pad="+"/>
<connect gate="G$1" pin="-" pad="-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="0805" package="0805">
<connects>
<connect gate="G$1" pin="+" pad="+"/>
<connect gate="G$1" pin="-" pad="-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GNDA">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<wire x1="-1.0922" y1="-0.508" x2="1.0922" y2="-0.508" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="GNDA" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="+3V3">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+3V3" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GNDA" prefix="GND">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GNDA" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+3V3" prefix="+3V3">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="+3V3" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="pinhead">
<description>&lt;b&gt;Pin Header Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="2X02">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-2.54" y1="-1.905" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="0" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.905" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.905" x2="-2.54" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.905" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0" y2="1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="1.905" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="2.54" y2="1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="1.905" x2="0" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.54" y1="1.905" x2="2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<pad name="1" x="-1.27" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="2" x="-1.27" y="1.27" drill="1.016" shape="octagon"/>
<pad name="3" x="1.27" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="4" x="1.27" y="1.27" drill="1.016" shape="octagon"/>
<text x="-2.54" y="3.175" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-4.445" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.524" y1="-1.524" x2="-1.016" y2="-1.016" layer="51"/>
<rectangle x1="-1.524" y1="1.016" x2="-1.016" y2="1.524" layer="51"/>
<rectangle x1="1.016" y1="1.016" x2="1.524" y2="1.524" layer="51"/>
<rectangle x1="1.016" y1="-1.524" x2="1.524" y2="-1.016" layer="51"/>
</package>
<package name="2X02/90">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-2.54" y1="-1.905" x2="0" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.905" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="6.985" x2="-1.27" y2="1.27" width="0.762" layer="21"/>
<wire x1="0" y1="-1.905" x2="2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="6.985" x2="1.27" y2="1.27" width="0.762" layer="21"/>
<pad name="2" x="-1.27" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="4" x="1.27" y="-3.81" drill="1.016" shape="octagon"/>
<pad name="1" x="-1.27" y="-6.35" drill="1.016" shape="octagon"/>
<pad name="3" x="1.27" y="-6.35" drill="1.016" shape="octagon"/>
<text x="-3.175" y="-3.81" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="4.445" y="-3.81" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-1.651" y1="0.635" x2="-0.889" y2="1.143" layer="21"/>
<rectangle x1="0.889" y1="0.635" x2="1.651" y2="1.143" layer="21"/>
<rectangle x1="-1.651" y1="-2.921" x2="-0.889" y2="-1.905" layer="21"/>
<rectangle x1="0.889" y1="-2.921" x2="1.651" y2="-1.905" layer="21"/>
<rectangle x1="-1.651" y1="-5.461" x2="-0.889" y2="-4.699" layer="21"/>
<rectangle x1="-1.651" y1="-4.699" x2="-0.889" y2="-2.921" layer="51"/>
<rectangle x1="0.889" y1="-4.699" x2="1.651" y2="-2.921" layer="51"/>
<rectangle x1="0.889" y1="-5.461" x2="1.651" y2="-4.699" layer="21"/>
</package>
<package name="1X01">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="-1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" shape="octagon"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="1X02">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-1.905" y1="1.27" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.635" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-0.635" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="2.54" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.635" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-2.6162" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.524" y1="-0.254" x2="-1.016" y2="0.254" layer="51"/>
<rectangle x1="1.016" y1="-0.254" x2="1.524" y2="0.254" layer="51"/>
</package>
<package name="1X02/90">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-2.54" y1="-1.905" x2="0" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.905" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="-2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0.635" x2="-2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="6.985" x2="-1.27" y2="1.27" width="0.762" layer="21"/>
<wire x1="0" y1="-1.905" x2="2.54" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="0.635" x2="0" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="6.985" x2="1.27" y2="1.27" width="0.762" layer="21"/>
<pad name="1" x="-1.27" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="1.27" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<text x="-3.175" y="-3.81" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="4.445" y="-3.81" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-1.651" y1="0.635" x2="-0.889" y2="1.143" layer="21"/>
<rectangle x1="0.889" y1="0.635" x2="1.651" y2="1.143" layer="21"/>
<rectangle x1="-1.651" y1="-2.921" x2="-0.889" y2="-1.905" layer="21"/>
<rectangle x1="0.889" y1="-2.921" x2="1.651" y2="-1.905" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="PINH2X2">
<wire x1="-8.89" y1="-2.54" x2="6.35" y2="-2.54" width="0.4064" layer="94"/>
<wire x1="6.35" y1="-2.54" x2="6.35" y2="5.08" width="0.4064" layer="94"/>
<wire x1="6.35" y1="5.08" x2="-8.89" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-8.89" y1="5.08" x2="-8.89" y2="-2.54" width="0.4064" layer="94"/>
<text x="-8.89" y="5.715" size="1.778" layer="95">&gt;NAME</text>
<text x="-8.89" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-5.08" y="2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="2" x="2.54" y="2.54" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
<pin name="3" x="-5.08" y="0" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="4" x="2.54" y="0" visible="pad" length="short" direction="pas" function="dot" rot="R180"/>
</symbol>
<symbol name="PINHD1">
<wire x1="-6.35" y1="-2.54" x2="1.27" y2="-2.54" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="1.27" y2="2.54" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="-6.35" y2="2.54" width="0.4064" layer="94"/>
<wire x1="-6.35" y1="2.54" x2="-6.35" y2="-2.54" width="0.4064" layer="94"/>
<text x="-6.35" y="3.175" size="1.778" layer="95">&gt;NAME</text>
<text x="-6.35" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-2.54" y="0" visible="pad" length="short" direction="pas" function="dot"/>
</symbol>
<symbol name="PINHD2">
<wire x1="-6.35" y1="-2.54" x2="1.27" y2="-2.54" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="1.27" y2="5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="5.08" x2="-6.35" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-6.35" y1="5.08" x2="-6.35" y2="-2.54" width="0.4064" layer="94"/>
<text x="-6.35" y="5.715" size="1.778" layer="95">&gt;NAME</text>
<text x="-6.35" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-2.54" y="2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="2" x="-2.54" y="0" visible="pad" length="short" direction="pas" function="dot"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="PINHD-2X2" prefix="JP" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="PINH2X2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="2X02">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="/90" package="2X02/90">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
<connect gate="A" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="PINHD-1X1" prefix="JP" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="PINHD1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="1X01">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="PINHD-1X2" prefix="JP" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="PINHD2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="1X02">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="/90" package="1X02/90">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U1" library="filip" deviceset="OSCILLATOR" device="SMD"/>
<part name="U$1" library="filip" deviceset="SPH0641LU4H-1" device=""/>
<part name="U$2" library="filip" deviceset="SPH0641LU4H-1" device=""/>
<part name="C1" library="filip" deviceset="CPOL" device="0805" value="0.1uF"/>
<part name="C2" library="filip" deviceset="CPOL" device="0805" value="0.1uF"/>
<part name="GND1" library="supply1" deviceset="GNDA" device=""/>
<part name="GND2" library="supply1" deviceset="GNDA" device=""/>
<part name="GND3" library="supply1" deviceset="GNDA" device=""/>
<part name="+3V1" library="supply1" deviceset="+3V3" device=""/>
<part name="+3V2" library="supply1" deviceset="+3V3" device=""/>
<part name="+3V3" library="supply1" deviceset="+3V3" device=""/>
<part name="IO1" library="pinhead" deviceset="PINHD-2X2" device=""/>
<part name="IO2" library="pinhead" deviceset="PINHD-2X2" device=""/>
<part name="GND" library="pinhead" deviceset="PINHD-1X1" device=""/>
<part name="POW" library="pinhead" deviceset="PINHD-1X2" device=""/>
<part name="GND4" library="supply1" deviceset="GNDA" device=""/>
<part name="C3" library="filip" deviceset="CPOL" device="A" value="4.7uF"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U1" gate="G$1" x="68.58" y="86.36"/>
<instance part="U$1" gate="G$1" x="111.76" y="101.6"/>
<instance part="U$2" gate="G$1" x="111.76" y="71.12"/>
<instance part="C1" gate="G$1" x="48.26" y="83.82" rot="R90"/>
<instance part="C2" gate="G$1" x="104.14" y="68.58" rot="R90"/>
<instance part="GND1" gate="1" x="43.18" y="66.04"/>
<instance part="GND2" gate="1" x="99.06" y="50.8"/>
<instance part="GND3" gate="1" x="99.06" y="93.98"/>
<instance part="+3V1" gate="G$1" x="43.18" y="124.46"/>
<instance part="+3V2" gate="G$1" x="101.6" y="119.38"/>
<instance part="+3V3" gate="G$1" x="101.6" y="86.36"/>
<instance part="IO1" gate="A" x="152.4" y="99.06"/>
<instance part="IO2" gate="A" x="152.4" y="68.58"/>
<instance part="GND" gate="G$1" x="157.48" y="55.88"/>
<instance part="POW" gate="G$1" x="55.88" y="109.22"/>
<instance part="GND4" gate="1" x="53.34" y="101.6"/>
<instance part="C3" gate="G$1" x="33.02" y="81.28" rot="R90"/>
</instances>
<busses>
</busses>
<nets>
<net name="GNDA" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="GND"/>
<wire x1="81.28" y1="83.82" x2="81.28" y2="68.58" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="-"/>
<wire x1="81.28" y1="68.58" x2="43.18" y2="68.58" width="0.1524" layer="91"/>
<wire x1="43.18" y1="68.58" x2="43.18" y2="81.28" width="0.1524" layer="91"/>
<pinref part="GND1" gate="1" pin="GNDA"/>
<junction x="43.18" y="68.58"/>
<pinref part="C3" gate="G$1" pin="-"/>
<wire x1="43.18" y1="68.58" x2="27.94" y2="68.58" width="0.1524" layer="91"/>
<wire x1="27.94" y1="68.58" x2="27.94" y2="78.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U$1" gate="G$1" pin="GND"/>
<pinref part="GND3" gate="1" pin="GNDA"/>
<wire x1="101.6" y1="96.52" x2="99.06" y2="96.52" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND2" gate="1" pin="GNDA"/>
<pinref part="C2" gate="G$1" pin="-"/>
<wire x1="99.06" y1="53.34" x2="99.06" y2="55.88" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="GND"/>
<wire x1="99.06" y1="55.88" x2="99.06" y2="66.04" width="0.1524" layer="91"/>
<wire x1="101.6" y1="66.04" x2="99.06" y2="66.04" width="0.1524" layer="91"/>
<junction x="99.06" y="66.04"/>
<pinref part="GND" gate="G$1" pin="1"/>
<wire x1="154.94" y1="55.88" x2="99.06" y2="55.88" width="0.1524" layer="91"/>
<junction x="99.06" y="55.88"/>
</segment>
<segment>
<pinref part="POW" gate="G$1" pin="2"/>
<pinref part="GND4" gate="1" pin="GNDA"/>
<wire x1="53.34" y1="109.22" x2="53.34" y2="104.14" width="0.1524" layer="91"/>
</segment>
</net>
<net name="+3V3" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="VCC"/>
<pinref part="C1" gate="G$1" pin="+"/>
<wire x1="55.88" y1="88.9" x2="43.18" y2="88.9" width="0.1524" layer="91"/>
<pinref part="+3V1" gate="G$1" pin="+3V3"/>
<wire x1="43.18" y1="88.9" x2="43.18" y2="111.76" width="0.1524" layer="91"/>
<junction x="43.18" y="88.9"/>
<pinref part="U1" gate="G$1" pin="EN"/>
<wire x1="43.18" y1="111.76" x2="43.18" y2="121.92" width="0.1524" layer="91"/>
<wire x1="55.88" y1="83.82" x2="55.88" y2="88.9" width="0.1524" layer="91"/>
<junction x="55.88" y="88.9"/>
<pinref part="POW" gate="G$1" pin="1"/>
<wire x1="53.34" y1="111.76" x2="43.18" y2="111.76" width="0.1524" layer="91"/>
<junction x="43.18" y="111.76"/>
<pinref part="C3" gate="G$1" pin="+"/>
<wire x1="43.18" y1="88.9" x2="27.94" y2="88.9" width="0.1524" layer="91"/>
<wire x1="27.94" y1="88.9" x2="27.94" y2="86.36" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="U$1" gate="G$1" pin="VDD"/>
<pinref part="+3V2" gate="G$1" pin="+3V3"/>
<wire x1="101.6" y1="109.22" x2="101.6" y2="116.84" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="SELECT"/>
<wire x1="121.92" y1="106.68" x2="124.46" y2="106.68" width="0.1524" layer="91"/>
<wire x1="124.46" y1="106.68" x2="124.46" y2="116.84" width="0.1524" layer="91"/>
<wire x1="124.46" y1="116.84" x2="101.6" y2="116.84" width="0.1524" layer="91"/>
<junction x="101.6" y="116.84"/>
</segment>
<segment>
<pinref part="U$2" gate="G$1" pin="VDD"/>
<pinref part="+3V3" gate="G$1" pin="+3V3"/>
<wire x1="101.6" y1="78.74" x2="101.6" y2="83.82" width="0.1524" layer="91"/>
<pinref part="C2" gate="G$1" pin="+"/>
<wire x1="101.6" y1="78.74" x2="99.06" y2="78.74" width="0.1524" layer="91"/>
<wire x1="99.06" y1="78.74" x2="99.06" y2="73.66" width="0.1524" layer="91"/>
<junction x="101.6" y="78.74"/>
<pinref part="U$2" gate="G$1" pin="SELECT"/>
<wire x1="121.92" y1="76.2" x2="121.92" y2="83.82" width="0.1524" layer="91"/>
<wire x1="121.92" y1="83.82" x2="101.6" y2="83.82" width="0.1524" layer="91"/>
<junction x="101.6" y="83.82"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="DATA"/>
<pinref part="IO1" gate="A" pin="1"/>
<wire x1="121.92" y1="101.6" x2="147.32" y2="101.6" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="IO1" gate="A" pin="3"/>
<wire x1="147.32" y1="99.06" x2="127" y2="99.06" width="0.1524" layer="91"/>
<wire x1="127" y1="99.06" x2="127" y2="96.52" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="CLK"/>
<wire x1="127" y1="96.52" x2="121.92" y2="96.52" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="U1" gate="G$1" pin="OUT"/>
<wire x1="81.28" y1="88.9" x2="162.56" y2="88.9" width="0.1524" layer="91"/>
<wire x1="162.56" y1="88.9" x2="162.56" y2="99.06" width="0.1524" layer="91"/>
<pinref part="IO1" gate="A" pin="2"/>
<wire x1="162.56" y1="99.06" x2="162.56" y2="101.6" width="0.1524" layer="91"/>
<wire x1="162.56" y1="101.6" x2="154.94" y2="101.6" width="0.1524" layer="91"/>
<pinref part="IO1" gate="A" pin="4"/>
<wire x1="154.94" y1="99.06" x2="162.56" y2="99.06" width="0.1524" layer="91"/>
<junction x="162.56" y="99.06"/>
<wire x1="162.56" y1="88.9" x2="162.56" y2="71.12" width="0.1524" layer="91"/>
<junction x="162.56" y="88.9"/>
<pinref part="IO2" gate="A" pin="2"/>
<wire x1="162.56" y1="71.12" x2="154.94" y2="71.12" width="0.1524" layer="91"/>
<wire x1="162.56" y1="71.12" x2="162.56" y2="68.58" width="0.1524" layer="91"/>
<junction x="162.56" y="71.12"/>
<pinref part="IO2" gate="A" pin="4"/>
<wire x1="162.56" y1="68.58" x2="154.94" y2="68.58" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="IO2" gate="A" pin="1"/>
<pinref part="U$2" gate="G$1" pin="DATA"/>
<wire x1="147.32" y1="71.12" x2="121.92" y2="71.12" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="IO2" gate="A" pin="3"/>
<wire x1="147.32" y1="68.58" x2="127" y2="68.58" width="0.1524" layer="91"/>
<wire x1="127" y1="68.58" x2="127" y2="66.04" width="0.1524" layer="91"/>
<pinref part="U$2" gate="G$1" pin="CLK"/>
<wire x1="127" y1="66.04" x2="121.92" y2="66.04" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
