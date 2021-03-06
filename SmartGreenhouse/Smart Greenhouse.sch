<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="24" fill="1" visible="no" active="no"/>
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
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="ESP32-DEVKITV1">
<packages>
<package name="ESP32-DEVKITV1">
<pad name="1" x="-22.87" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="2" x="-20.33" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="3" x="-17.79" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="4" x="-15.25" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="5" x="-12.71" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="6" x="-10.17" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="7" x="-7.63" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="8" x="-5.09" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="9" x="-2.55" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="10" x="-0.01" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="11" x="2.53" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="12" x="5.07" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="13" x="7.61" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="14" x="10.15" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="15" x="12.69" y="-13.54" drill="1" diameter="1.9304"/>
<pad name="30" x="-22.87" y="11.23" drill="1" diameter="1.9304"/>
<pad name="29" x="-20.33" y="11.23" drill="1" diameter="1.9304"/>
<pad name="28" x="-17.79" y="11.23" drill="1" diameter="1.9304"/>
<pad name="27" x="-15.25" y="11.23" drill="1" diameter="1.9304"/>
<pad name="26" x="-12.71" y="11.23" drill="1" diameter="1.9304"/>
<pad name="25" x="-10.17" y="11.23" drill="1" diameter="1.9304"/>
<pad name="24" x="-7.63" y="11.23" drill="1" diameter="1.9304"/>
<pad name="23" x="-5.09" y="11.23" drill="1" diameter="1.9304"/>
<pad name="22" x="-2.55" y="11.23" drill="1" diameter="1.9304"/>
<pad name="21" x="-0.01" y="11.23" drill="1" diameter="1.9304"/>
<pad name="20" x="2.53" y="11.23" drill="1" diameter="1.9304"/>
<pad name="19" x="5.07" y="11.23" drill="1" diameter="1.9304"/>
<pad name="18" x="7.61" y="11.23" drill="1" diameter="1.9304"/>
<pad name="17" x="10.15" y="11.23" drill="1" diameter="1.9304"/>
<pad name="16" x="12.69" y="11.23" drill="1" diameter="1.9304"/>
<text x="-22.21" y="-11.2" size="1.016" layer="21" rot="R90">3V3</text>
<text x="-19.67" y="-11.2" size="1.016" layer="21" rot="R90">GND</text>
<text x="-17.13" y="-11.2" size="1.016" layer="21" rot="R90">IO15</text>
<text x="-14.59" y="-11.2" size="1.016" layer="21" rot="R90">IO2</text>
<text x="-12.05" y="-11.2" size="1.016" layer="21" rot="R90">IO4</text>
<text x="-9.51" y="-11.2" size="1.016" layer="21" rot="R90">IO16</text>
<text x="-6.97" y="-11.2" size="1.016" layer="21" rot="R90">IO17</text>
<text x="-4.43" y="-11.2" size="1.016" layer="21" rot="R90">IO5</text>
<text x="-1.89" y="-11.2" size="1.016" layer="21" rot="R90">IO18</text>
<text x="0.65" y="-11.2" size="1.016" layer="21" rot="R90">IO19</text>
<text x="3.19" y="-11.2" size="1.016" layer="21" rot="R90">IO21</text>
<text x="5.73" y="-11.2" size="1.016" layer="21" rot="R90">IO3</text>
<text x="8.27" y="-11.2" size="1.016" layer="21" rot="R90">IO1</text>
<text x="10.81" y="-11.2" size="1.016" layer="21" rot="R90">IO22</text>
<text x="13.35" y="-11.2" size="1.016" layer="21" rot="R90">IO23</text>
<text x="-22.19" y="6.52" size="1.016" layer="21" rot="R90">VIN</text>
<text x="-19.65" y="6.52" size="1.016" layer="21" rot="R90">GND</text>
<text x="-17.11" y="6.52" size="1.016" layer="21" rot="R90">IO13</text>
<text x="-14.57" y="6.52" size="1.016" layer="21" rot="R90">IO12</text>
<text x="-12.03" y="6.52" size="1.016" layer="21" rot="R90">IO14</text>
<text x="-9.49" y="6.52" size="1.016" layer="21" rot="R90">IO27</text>
<text x="-6.95" y="6.52" size="1.016" layer="21" rot="R90">IO26</text>
<text x="-4.41" y="6.52" size="1.016" layer="21" rot="R90">IO25</text>
<text x="-1.87" y="6.52" size="1.016" layer="21" rot="R90">IO33</text>
<text x="0.67" y="6.52" size="1.016" layer="21" rot="R90">IO32</text>
<text x="3.21" y="6.52" size="1.016" layer="21" rot="R90">IO35</text>
<text x="5.75" y="6.52" size="1.016" layer="21" rot="R90">IO34</text>
<text x="8.29" y="6.52" size="1.016" layer="21" rot="R90">VN</text>
<text x="10.83" y="6.52" size="1.016" layer="21" rot="R90">VP</text>
<text x="13.37" y="6.52" size="1.016" layer="21" rot="R90">EN</text>
<text x="-4.93" y="-3.18" size="1.9304" layer="21">ESP32-Devkit V1</text>
<wire x1="-33" y1="12.7" x2="19" y2="12.7" width="0.254" layer="21"/>
<wire x1="19" y1="12.7" x2="19" y2="-15.2" width="0.254" layer="21"/>
<wire x1="19" y1="-15.2" x2="-33" y2="-15.2" width="0.254" layer="21"/>
<wire x1="-33" y1="-15.2" x2="-33" y2="12.7" width="0.254" layer="21"/>
<text x="-24.13" y="13.97" size="1.27" layer="21">&gt;NAME</text>
<text x="5" y="-17.24" size="1.27" layer="27">ESP32-DEVKITV1</text>
</package>
</packages>
<symbols>
<symbol name="ESP32-DEVKITV1">
<wire x1="-25.4" y1="-12.7" x2="-25.4" y2="12.7" width="0.254" layer="94"/>
<wire x1="-25.4" y1="12.7" x2="16" y2="12.7" width="0.254" layer="94"/>
<wire x1="16" y1="12.7" x2="16" y2="-12.7" width="0.254" layer="94"/>
<wire x1="16" y1="-12.7" x2="-25.4" y2="-12.7" width="0.254" layer="94"/>
<pin name="3V3" x="-22.86" y="-17.78" length="middle" rot="R90"/>
<pin name="GND" x="-20.32" y="-17.78" length="middle" rot="R90"/>
<pin name="IO15" x="-17.78" y="-17.78" length="middle" rot="R90"/>
<pin name="IO2" x="-15.24" y="-17.78" length="middle" rot="R90"/>
<pin name="IO4" x="-12.7" y="-17.78" length="middle" rot="R90"/>
<pin name="IO16" x="-10.16" y="-17.78" length="middle" rot="R90"/>
<pin name="IO17" x="-7.62" y="-17.78" length="middle" rot="R90"/>
<pin name="IO5" x="-5.08" y="-17.78" length="middle" rot="R90"/>
<pin name="IO18" x="-2.54" y="-17.78" length="middle" rot="R90"/>
<pin name="IO19" x="0" y="-17.78" length="middle" rot="R90"/>
<pin name="IO21" x="2.54" y="-17.78" length="middle" rot="R90"/>
<pin name="IO3" x="5.08" y="-17.78" length="middle" rot="R90"/>
<pin name="IO1" x="7.62" y="-17.78" length="middle" rot="R90"/>
<pin name="IO22" x="10.16" y="-17.78" length="middle" rot="R90"/>
<pin name="IO23" x="12.7" y="-17.78" length="middle" rot="R90"/>
<pin name="EN" x="12.7" y="17.78" length="middle" rot="R270"/>
<pin name="VP" x="10.16" y="17.78" length="middle" rot="R270"/>
<pin name="VN" x="7.62" y="17.78" length="middle" rot="R270"/>
<pin name="IO34" x="5.08" y="17.78" length="middle" rot="R270"/>
<pin name="IO35" x="2.54" y="17.78" length="middle" rot="R270"/>
<pin name="IO32" x="0" y="17.78" length="middle" rot="R270"/>
<pin name="IO33" x="-2.54" y="17.78" length="middle" rot="R270"/>
<pin name="IO25" x="-5.08" y="17.78" length="middle" rot="R270"/>
<pin name="IO26" x="-7.62" y="17.78" length="middle" rot="R270"/>
<pin name="IO27" x="-10.16" y="17.78" length="middle" rot="R270"/>
<pin name="IO14" x="-12.7" y="17.78" length="middle" rot="R270"/>
<pin name="IO12" x="-15.24" y="17.78" length="middle" rot="R270"/>
<pin name="IO13" x="-17.78" y="17.78" length="middle" rot="R270"/>
<pin name="GND1" x="-20.32" y="17.78" length="middle" rot="R270"/>
<pin name="VIN" x="-22.86" y="17.78" length="middle" rot="R270"/>
<text x="-26.67" y="5.08" size="1.27" layer="95" rot="R90">&gt;NAME</text>
<text x="18.4" y="-12.7" size="1.27" layer="96" rot="R90">ESP32-DEVKITV1</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="ESP32DEVKITV1">
<gates>
<gate name="G$1" symbol="ESP32-DEVKITV1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ESP32-DEVKITV1">
<connects>
<connect gate="G$1" pin="3V3" pad="1"/>
<connect gate="G$1" pin="EN" pad="16"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="GND1" pad="28"/>
<connect gate="G$1" pin="IO1" pad="13"/>
<connect gate="G$1" pin="IO12" pad="27"/>
<connect gate="G$1" pin="IO13" pad="28"/>
<connect gate="G$1" pin="IO14" pad="26"/>
<connect gate="G$1" pin="IO15" pad="3"/>
<connect gate="G$1" pin="IO16" pad="6"/>
<connect gate="G$1" pin="IO17" pad="7"/>
<connect gate="G$1" pin="IO18" pad="9"/>
<connect gate="G$1" pin="IO19" pad="10"/>
<connect gate="G$1" pin="IO2" pad="4"/>
<connect gate="G$1" pin="IO21" pad="11"/>
<connect gate="G$1" pin="IO22" pad="14"/>
<connect gate="G$1" pin="IO23" pad="15"/>
<connect gate="G$1" pin="IO25" pad="23"/>
<connect gate="G$1" pin="IO26" pad="24"/>
<connect gate="G$1" pin="IO27" pad="24"/>
<connect gate="G$1" pin="IO3" pad="12"/>
<connect gate="G$1" pin="IO32" pad="21"/>
<connect gate="G$1" pin="IO33" pad="22"/>
<connect gate="G$1" pin="IO34" pad="19"/>
<connect gate="G$1" pin="IO35" pad="20"/>
<connect gate="G$1" pin="IO4" pad="5"/>
<connect gate="G$1" pin="IO5" pad="8"/>
<connect gate="G$1" pin="VIN" pad="30"/>
<connect gate="G$1" pin="VN" pad="18"/>
<connect gate="G$1" pin="VP" pad="17"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-wago255" urn="urn:adsk.eagle:library:198">
<description>&lt;b&gt;Wago Connectors&lt;/b&gt;&lt;p&gt;
Types 233 and 234&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="255-404-5" urn="urn:adsk.eagle:footprint:10872/1" library_version="1">
<description>&lt;b&gt;Grid 5.0 mm&lt;/b&gt;</description>
<wire x1="-11.05" y1="6.05" x2="-10.7" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="6.05" x2="-10.7" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="-0.55" x2="-11.05" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-11.05" y1="-0.55" x2="-11.05" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-11.05" y1="-2.35" x2="-10.7" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="-2.35" x2="-10.7" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="-4.35" x2="-11.05" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-11.05" y1="-4.35" x2="-11.05" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-11.05" y1="-5.35" x2="-10.7" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="-5.35" x2="-10.7" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="-6.2" x2="-10.05" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-10.05" y1="-6.2" x2="-9.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-9.85" y1="-6.2" x2="9.9" y2="-6.2" width="0.2032" layer="51"/>
<wire x1="9.9" y1="-6.2" x2="10.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="10.1" y1="-6.2" x2="11.65" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-6.2" x2="11.65" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-4.35" x2="11.65" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-3.65" x2="11.65" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-0.55" x2="11.65" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="-3.65" x2="-10" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-10" y1="-3.65" x2="9.9" y2="-3.65" width="0.2032" layer="51"/>
<wire x1="9.95" y1="-3.65" x2="11.65" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-8.45" y1="0.75" x2="-8.45" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-8.45" y1="-3.05" x2="-6.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-6.5" y1="-3.05" x2="-6.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-6.5" y1="0.75" x2="-8.45" y2="0.75" width="0.2032" layer="51"/>
<wire x1="11.3" y1="6.05" x2="11.65" y2="6.05" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-0.55" x2="11.3" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="11.3" y1="-0.55" x2="11.3" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.3" y1="-2.35" x2="11.65" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-4.35" x2="11.3" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="11.3" y1="-4.35" x2="11.3" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="11.3" y1="-5.35" x2="11.65" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-3.5" y1="0.75" x2="-3.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-3.5" y1="-3.05" x2="-1.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="-3.05" x2="-1.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="0.75" x2="-3.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-10.7" y1="7.575" x2="-10.7" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="7.05" x2="-11.05" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-11.05" y1="7.05" x2="-11.05" y2="6.05" width="0.2032" layer="21"/>
<wire x1="11.65" y1="7.05" x2="11.65" y2="7.55" width="0.2032" layer="21"/>
<wire x1="-9.85" y1="7.55" x2="-9.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-9.8" y1="11.85" x2="-9.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-9.8" y1="9.35" x2="-8.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-8.1" y1="9.35" x2="-6.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-6.9" y1="9.35" x2="-5.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-5.2" y1="9.35" x2="-5.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-5.2" y1="11.85" x2="-9.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-5.1" y1="7.55" x2="-5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-5.1" y1="2.65" x2="-5.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="7.55" x2="-0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="2.65" x2="-0.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="11.65" y1="7.05" x2="11.3" y2="7.05" width="0.2032" layer="21"/>
<wire x1="11.3" y1="7.05" x2="11.3" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-8.85" y1="11.35" x2="-8.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-8.85" y1="9.85" x2="-6.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-6.15" y1="9.85" x2="-6.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-6.15" y1="11.35" x2="-8.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-8.1" y1="9.35" x2="-8.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-8.1" y1="6.7" x2="-6.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-6.9" y1="6.7" x2="-6.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-10.7" y1="7.575" x2="-8.125" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-4.8" y1="11.85" x2="-4.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-4.8" y1="9.35" x2="-3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="9.35" x2="-1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.9" y1="9.35" x2="-0.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-0.2" y1="9.35" x2="-0.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-0.2" y1="11.85" x2="-4.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-3.85" y1="11.35" x2="-3.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-3.85" y1="9.85" x2="-1.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-1.15" y1="9.85" x2="-1.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-1.15" y1="11.35" x2="-3.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="9.35" x2="-3.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="6.7" x2="-1.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-1.9" y1="6.7" x2="-1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-6.825" y1="7.575" x2="-3.125" y2="7.575" width="0.2032" layer="21"/>
<wire x1="8.125" y1="7.575" x2="11.625" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-4.9" y1="7.55" x2="-4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-4.9" y1="2.65" x2="-4.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-10.05" y1="7.55" x2="-10.05" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-10.65" y1="2.65" x2="-5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-4.9" y1="2.65" x2="-0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="2.65" x2="0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="0.1" y1="2.65" x2="4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="9.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="9.9" y1="2.65" x2="10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="10.1" y1="2.65" x2="11.6" y2="2.65" width="0.2032" layer="21"/>
<wire x1="1.5" y1="0.75" x2="1.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="1.5" y1="-3.05" x2="3.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="3.5" y1="-3.05" x2="3.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="3.5" y1="0.75" x2="1.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="0.2" y1="11.85" x2="0.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="0.2" y1="9.35" x2="1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="1.9" y1="9.35" x2="3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.1" y1="9.35" x2="4.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="4.8" y1="9.35" x2="4.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="4.8" y1="11.85" x2="0.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="1.15" y1="11.35" x2="1.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="1.15" y1="9.85" x2="3.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="3.85" y1="9.85" x2="3.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="3.85" y1="11.35" x2="1.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="1.9" y1="9.35" x2="1.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="1.9" y1="6.7" x2="3.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="3.1" y1="6.7" x2="3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.875" y1="7.575" x2="1.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="0.1" y1="7.55" x2="0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="0.1" y1="2.65" x2="0.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="10.1" y1="7.55" x2="10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="10.1" y1="2.65" x2="10.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="9.9" y1="2.65" x2="9.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="9.9" y1="7.55" x2="9.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="6.5" y1="0.75" x2="6.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="6.5" y1="-3.05" x2="8.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="8.5" y1="-3.05" x2="8.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="8.5" y1="0.75" x2="6.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="5.2" y1="11.85" x2="5.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="5.2" y1="9.35" x2="6.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="6.9" y1="9.35" x2="8.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="8.1" y1="9.35" x2="9.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="9.8" y1="9.35" x2="9.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="9.8" y1="11.85" x2="5.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="6.15" y1="11.35" x2="6.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="6.15" y1="9.85" x2="8.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="8.85" y1="9.85" x2="8.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="8.85" y1="11.35" x2="6.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="6.9" y1="9.35" x2="6.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="6.9" y1="6.7" x2="8.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="8.1" y1="6.7" x2="8.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.125" y1="7.575" x2="6.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="5.1" y1="7.55" x2="5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="5.1" y1="2.65" x2="5.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="4.9" y1="7.55" x2="4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="4.9" y2="-6.2" width="0.2032" layer="21"/>
<pad name="A1" x="-7.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B1" x="-7.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A2" x="-2.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B2" x="-2.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A3" x="2.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B3" x="2.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A4" x="7.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B4" x="7.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<text x="-7.75" y="1.35" size="0.8128" layer="21">1</text>
<text x="-11.535" y="-5.855" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="13.9" y="-5.75" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
</package>
<package name="255-408-5" urn="urn:adsk.eagle:footprint:10876/1" library_version="1">
<description>&lt;b&gt;Grid 5.0 mm&lt;/b&gt;</description>
<wire x1="-21.05" y1="6.05" x2="-20.7" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="6.05" x2="-20.7" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="-0.55" x2="-21.05" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-21.05" y1="-0.55" x2="-21.05" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-21.05" y1="-2.35" x2="-20.7" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="-2.35" x2="-20.7" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="-4.35" x2="-21.05" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-21.05" y1="-4.35" x2="-21.05" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-21.05" y1="-5.35" x2="-20.7" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="-5.35" x2="-20.7" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="-6.2" x2="-20.05" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-20.05" y1="-6.2" x2="-19.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-19.85" y1="-6.2" x2="19.9" y2="-6.2" width="0.2032" layer="51"/>
<wire x1="19.9" y1="-6.2" x2="20.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="20.1" y1="-6.2" x2="21.65" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="21.65" y1="-6.2" x2="21.65" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="21.65" y1="-4.35" x2="21.65" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="21.65" y1="-3.65" x2="21.65" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="21.65" y1="-0.55" x2="21.65" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="-3.65" x2="-20" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-20" y1="-3.65" x2="19.9" y2="-3.65" width="0.2032" layer="51"/>
<wire x1="19.95" y1="-3.65" x2="21.65" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-18.45" y1="0.75" x2="-18.45" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-18.45" y1="-3.05" x2="-16.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-16.5" y1="-3.05" x2="-16.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-16.5" y1="0.75" x2="-18.45" y2="0.75" width="0.2032" layer="51"/>
<wire x1="21.3" y1="6.05" x2="21.65" y2="6.05" width="0.2032" layer="21"/>
<wire x1="21.65" y1="-0.55" x2="21.3" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="21.3" y1="-0.55" x2="21.3" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="21.3" y1="-2.35" x2="21.65" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="21.65" y1="-4.35" x2="21.3" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="21.3" y1="-4.35" x2="21.3" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="21.3" y1="-5.35" x2="21.65" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-13.5" y1="0.75" x2="-13.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-13.5" y1="-3.05" x2="-11.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-11.5" y1="-3.05" x2="-11.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-11.5" y1="0.75" x2="-13.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-20.7" y1="7.575" x2="-20.7" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="7.05" x2="-21.05" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-21.05" y1="7.05" x2="-21.05" y2="6.05" width="0.2032" layer="21"/>
<wire x1="21.65" y1="7.05" x2="21.65" y2="7.55" width="0.2032" layer="21"/>
<wire x1="-19.85" y1="7.55" x2="-19.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-19.8" y1="11.85" x2="-19.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-19.8" y1="9.35" x2="-18.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-18.1" y1="9.35" x2="-16.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-16.9" y1="9.35" x2="-15.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-15.2" y1="9.35" x2="-15.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-15.2" y1="11.85" x2="-19.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-15.1" y1="7.55" x2="-15.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-15.1" y1="2.65" x2="-15.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-10.1" y1="7.55" x2="-10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-10.1" y1="2.65" x2="-10.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="21.65" y1="7.05" x2="21.3" y2="7.05" width="0.2032" layer="21"/>
<wire x1="21.3" y1="7.05" x2="21.3" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-18.85" y1="11.35" x2="-18.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-18.85" y1="9.85" x2="-16.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-16.15" y1="9.85" x2="-16.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-16.15" y1="11.35" x2="-18.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-18.1" y1="9.35" x2="-18.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-18.1" y1="6.7" x2="-16.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-16.9" y1="6.7" x2="-16.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-20.7" y1="7.575" x2="-18.125" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-14.8" y1="11.85" x2="-14.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-14.8" y1="9.35" x2="-13.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-13.1" y1="9.35" x2="-11.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-11.9" y1="9.35" x2="-10.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-10.2" y1="9.35" x2="-10.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-10.2" y1="11.85" x2="-14.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-13.85" y1="11.35" x2="-13.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-13.85" y1="9.85" x2="-11.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-11.15" y1="9.85" x2="-11.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-11.15" y1="11.35" x2="-13.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-13.1" y1="9.35" x2="-13.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-13.1" y1="6.7" x2="-11.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-11.9" y1="6.7" x2="-11.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-16.825" y1="7.575" x2="-13.125" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-14.9" y1="7.55" x2="-14.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-14.9" y1="2.65" x2="-14.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-20.05" y1="7.55" x2="-20.05" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-20.65" y1="2.65" x2="-15.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-14.9" y1="2.65" x2="-10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-10.1" y1="2.65" x2="-9.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-9.9" y1="2.65" x2="-5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-5.1" y1="2.65" x2="19.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="19.9" y1="2.65" x2="20.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="20.1" y1="2.65" x2="21.6" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-8.5" y1="0.75" x2="-8.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-8.5" y1="-3.05" x2="-6.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-6.5" y1="-3.05" x2="-6.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-6.5" y1="0.75" x2="-8.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-9.8" y1="11.85" x2="-9.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-9.8" y1="9.35" x2="-8.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-8.1" y1="9.35" x2="-6.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-6.9" y1="9.35" x2="-5.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-5.2" y1="9.35" x2="-5.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-5.2" y1="11.85" x2="-9.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-8.85" y1="11.35" x2="-8.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-8.85" y1="9.85" x2="-6.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-6.15" y1="9.85" x2="-6.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-6.15" y1="11.35" x2="-8.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-8.1" y1="9.35" x2="-8.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-8.1" y1="6.7" x2="-6.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-6.9" y1="6.7" x2="-6.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-11.875" y1="7.575" x2="-8.125" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-9.9" y1="7.55" x2="-9.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-9.9" y1="2.65" x2="-9.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="20.1" y1="7.55" x2="20.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="20.1" y1="2.65" x2="20.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="19.9" y1="2.65" x2="19.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="19.9" y1="7.55" x2="19.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-5.1" y1="2.65" x2="-4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-3.5" y1="0.75" x2="-3.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-3.5" y1="-3.05" x2="-1.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="-3.05" x2="-1.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="0.75" x2="-3.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-4.8" y1="11.85" x2="-4.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-4.8" y1="9.35" x2="-3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="9.35" x2="-1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.9" y1="9.35" x2="-0.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-0.2" y1="9.35" x2="-0.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-0.2" y1="11.85" x2="-4.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-3.85" y1="11.35" x2="-3.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-3.85" y1="9.85" x2="-1.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-1.15" y1="9.85" x2="-1.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-1.15" y1="11.35" x2="-3.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="9.35" x2="-3.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="6.7" x2="-1.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-1.9" y1="6.7" x2="-1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-6.875" y1="7.575" x2="-3.125" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-4.9" y1="7.55" x2="-4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-4.9" y1="2.65" x2="-4.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.1" y1="7.55" x2="-5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-5.1" y1="2.65" x2="-5.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.9" y1="2.65" x2="-0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="2.65" x2="0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="1.5" y1="0.75" x2="1.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="1.5" y1="-3.05" x2="3.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="3.5" y1="-3.05" x2="3.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="3.5" y1="0.75" x2="1.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="0.2" y1="11.85" x2="0.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="0.2" y1="9.35" x2="1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="1.9" y1="9.35" x2="3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.1" y1="9.35" x2="4.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="4.8" y1="9.35" x2="4.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="4.8" y1="11.85" x2="0.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="1.15" y1="11.35" x2="1.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="1.15" y1="9.85" x2="3.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="3.85" y1="9.85" x2="3.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="3.85" y1="11.35" x2="1.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="1.9" y1="9.35" x2="1.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="1.9" y1="6.7" x2="3.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="3.1" y1="6.7" x2="3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.875" y1="7.575" x2="1.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="0.1" y1="7.55" x2="0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="0.1" y1="2.65" x2="0.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="7.55" x2="-0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="2.65" x2="-0.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="0.1" y1="2.65" x2="4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="6.5" y1="0.75" x2="6.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="6.5" y1="-3.05" x2="8.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="8.5" y1="-3.05" x2="8.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="8.5" y1="0.75" x2="6.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="5.2" y1="11.85" x2="5.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="5.2" y1="9.35" x2="6.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="6.9" y1="9.35" x2="8.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="8.1" y1="9.35" x2="9.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="9.8" y1="9.35" x2="9.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="9.8" y1="11.85" x2="5.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="6.15" y1="11.35" x2="6.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="6.15" y1="9.85" x2="8.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="8.85" y1="9.85" x2="8.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="8.85" y1="11.35" x2="6.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="6.9" y1="9.35" x2="6.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="6.9" y1="6.7" x2="8.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="8.1" y1="6.7" x2="8.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.125" y1="7.575" x2="6.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="5.1" y1="7.55" x2="5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="5.1" y1="2.65" x2="5.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="4.9" y1="7.55" x2="4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="4.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="5.1" y1="2.65" x2="9.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="9.9" y1="2.65" x2="10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="11.5" y1="0.75" x2="11.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="11.5" y1="-3.05" x2="13.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="13.5" y1="-3.05" x2="13.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="13.5" y1="0.75" x2="11.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="10.2" y1="11.85" x2="10.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="10.2" y1="9.35" x2="11.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="11.9" y1="9.35" x2="13.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="13.1" y1="9.35" x2="14.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="14.8" y1="9.35" x2="14.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="14.8" y1="11.85" x2="10.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="11.15" y1="11.35" x2="11.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="11.15" y1="9.85" x2="13.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="13.85" y1="9.85" x2="13.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="13.85" y1="11.35" x2="11.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="11.9" y1="9.35" x2="11.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="11.9" y1="6.7" x2="13.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="13.1" y1="6.7" x2="13.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="8.125" y1="7.575" x2="11.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="10.1" y1="7.55" x2="10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="10.1" y1="2.65" x2="10.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="9.9" y1="7.55" x2="9.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="9.9" y1="2.65" x2="9.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="18.125" y1="7.575" x2="21.625" y2="7.575" width="0.2032" layer="21"/>
<wire x1="10.1" y1="2.65" x2="14.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="14.9" y1="2.65" x2="15.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="16.5" y1="0.75" x2="16.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="16.5" y1="-3.05" x2="18.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="18.5" y1="-3.05" x2="18.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="18.5" y1="0.75" x2="16.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="15.2" y1="11.85" x2="15.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="15.2" y1="9.35" x2="16.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="16.9" y1="9.35" x2="18.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="18.1" y1="9.35" x2="19.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="19.8" y1="9.35" x2="19.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="19.8" y1="11.85" x2="15.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="16.15" y1="11.35" x2="16.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="16.15" y1="9.85" x2="18.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="18.85" y1="9.85" x2="18.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="18.85" y1="11.35" x2="16.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="16.9" y1="9.35" x2="16.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="16.9" y1="6.7" x2="18.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="18.1" y1="6.7" x2="18.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="13.125" y1="7.575" x2="16.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="15.1" y1="7.55" x2="15.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="15.1" y1="2.65" x2="15.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="14.9" y1="7.55" x2="14.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="14.9" y1="2.65" x2="14.9" y2="-6.2" width="0.2032" layer="21"/>
<pad name="A1" x="-17.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B1" x="-17.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A2" x="-12.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B2" x="-12.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A3" x="-7.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B3" x="-7.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A4" x="-2.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B4" x="-2.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A5" x="2.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B5" x="2.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A6" x="7.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B6" x="7.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A7" x="12.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B7" x="12.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A8" x="17.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B8" x="17.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<text x="-17.75" y="1.35" size="0.8128" layer="21">1</text>
<text x="-21.535" y="-5.855" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="23.9" y="-5.75" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
</package>
<package name="255-403-5" urn="urn:adsk.eagle:footprint:10870/1" library_version="1">
<description>&lt;b&gt;Grid 5.0 mm&lt;/b&gt;</description>
<wire x1="-6.05" y1="6.05" x2="-5.7" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="6.05" x2="-5.7" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-0.55" x2="-6.05" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="-0.55" x2="-6.05" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="-2.35" x2="-5.7" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-2.35" x2="-5.7" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-4.35" x2="-6.05" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="-4.35" x2="-6.05" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="-5.35" x2="-5.7" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-5.35" x2="-5.7" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-6.2" x2="-5.05" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.05" y1="-6.2" x2="-4.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.85" y1="-6.2" x2="9.9" y2="-6.2" width="0.2032" layer="51"/>
<wire x1="9.9" y1="-6.2" x2="10.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="10.1" y1="-6.2" x2="11.65" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-6.2" x2="11.65" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-4.35" x2="11.65" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-3.65" x2="11.65" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-0.55" x2="11.65" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-3.65" x2="-5" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-5" y1="-3.65" x2="9.9" y2="-3.65" width="0.2032" layer="51"/>
<wire x1="9.95" y1="-3.65" x2="11.65" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-3.45" y1="0.75" x2="-3.45" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-3.45" y1="-3.05" x2="-1.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="-3.05" x2="-1.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="0.75" x2="-3.45" y2="0.75" width="0.2032" layer="51"/>
<wire x1="11.3" y1="6.05" x2="11.65" y2="6.05" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-0.55" x2="11.3" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="11.3" y1="-0.55" x2="11.3" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.3" y1="-2.35" x2="11.65" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.65" y1="-4.35" x2="11.3" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="11.3" y1="-4.35" x2="11.3" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="11.3" y1="-5.35" x2="11.65" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="1.5" y1="0.75" x2="1.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="1.5" y1="-3.05" x2="3.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="3.5" y1="-3.05" x2="3.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="3.5" y1="0.75" x2="1.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-5.7" y1="7.575" x2="-5.7" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="7.05" x2="-6.05" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="7.05" x2="-6.05" y2="6.05" width="0.2032" layer="21"/>
<wire x1="11.65" y1="7.05" x2="11.65" y2="7.55" width="0.2032" layer="21"/>
<wire x1="-4.85" y1="7.55" x2="-4.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.8" y1="11.85" x2="-4.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-4.8" y1="9.35" x2="-3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="9.35" x2="-1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.9" y1="9.35" x2="-0.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-0.2" y1="9.35" x2="-0.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-0.2" y1="11.85" x2="-4.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="7.55" x2="-0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="2.65" x2="-0.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="4.9" y1="7.55" x2="4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="4.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="11.65" y1="7.05" x2="11.3" y2="7.05" width="0.2032" layer="21"/>
<wire x1="11.3" y1="7.05" x2="11.3" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-3.85" y1="11.35" x2="-3.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-3.85" y1="9.85" x2="-1.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-1.15" y1="9.85" x2="-1.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-1.15" y1="11.35" x2="-3.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="9.35" x2="-3.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="6.7" x2="-1.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-1.9" y1="6.7" x2="-1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="7.575" x2="-3.125" y2="7.575" width="0.2032" layer="21"/>
<wire x1="0.2" y1="11.85" x2="0.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="0.2" y1="9.35" x2="1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="1.9" y1="9.35" x2="3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.1" y1="9.35" x2="4.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="4.8" y1="9.35" x2="4.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="4.8" y1="11.85" x2="0.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="1.15" y1="11.35" x2="1.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="1.15" y1="9.85" x2="3.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="3.85" y1="9.85" x2="3.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="3.85" y1="11.35" x2="1.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="1.9" y1="9.35" x2="1.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="1.9" y1="6.7" x2="3.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="3.1" y1="6.7" x2="3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.825" y1="7.575" x2="1.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="8.125" y1="7.575" x2="11.625" y2="7.575" width="0.2032" layer="21"/>
<wire x1="0.1" y1="7.55" x2="0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="0.1" y1="2.65" x2="0.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.05" y1="7.55" x2="-5.05" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.65" y1="2.65" x2="-0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="0.1" y1="2.65" x2="4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="5.1" y1="2.65" x2="9.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="9.9" y1="2.65" x2="10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="10.1" y1="2.65" x2="11.6" y2="2.65" width="0.2032" layer="21"/>
<wire x1="6.5" y1="0.75" x2="6.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="6.5" y1="-3.05" x2="8.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="8.5" y1="-3.05" x2="8.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="8.5" y1="0.75" x2="6.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="5.2" y1="11.85" x2="5.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="5.2" y1="9.35" x2="6.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="6.9" y1="9.35" x2="8.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="8.1" y1="9.35" x2="9.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="9.8" y1="9.35" x2="9.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="9.8" y1="11.85" x2="5.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="6.15" y1="11.35" x2="6.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="6.15" y1="9.85" x2="8.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="8.85" y1="9.85" x2="8.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="8.85" y1="11.35" x2="6.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="6.9" y1="9.35" x2="6.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="6.9" y1="6.7" x2="8.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="8.1" y1="6.7" x2="8.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.125" y1="7.575" x2="6.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="5.1" y1="7.55" x2="5.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="5.1" y1="2.65" x2="5.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="10.1" y1="7.55" x2="10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="10.1" y1="2.65" x2="10.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="9.9" y1="2.65" x2="9.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="9.9" y1="7.55" x2="9.9" y2="2.65" width="0.2032" layer="21"/>
<pad name="A1" x="-2.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B1" x="-2.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A2" x="2.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B2" x="2.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A3" x="7.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B3" x="7.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<text x="-2.75" y="1.35" size="0.8128" layer="21">1</text>
<text x="-6.535" y="-5.855" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="13.9" y="-5.75" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
</package>
<package name="255-403-508" urn="urn:adsk.eagle:footprint:10871/1" library_version="1">
<description>&lt;b&gt;Grid 5.0 mm&lt;/b&gt;</description>
<wire x1="-6.09" y1="6.05" x2="-5.74" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="6.05" x2="-5.74" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-0.55" x2="-6.09" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="-0.55" x2="-6.09" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="-2.35" x2="-5.74" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-2.35" x2="-5.74" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-4.35" x2="-6.09" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="-4.35" x2="-6.09" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="-5.35" x2="-5.74" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-5.35" x2="-5.74" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-6.2" x2="-5.09" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.09" y1="-6.2" x2="-4.89" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.89" y1="-6.2" x2="10.1" y2="-6.2" width="0.2032" layer="51"/>
<wire x1="-5.74" y1="-3.65" x2="-5.04" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-5.04" y1="-3.65" x2="10.1" y2="-3.65" width="0.2032" layer="51"/>
<wire x1="-3.49" y1="0.75" x2="-3.49" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-3.49" y1="-3.05" x2="-1.54" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-1.54" y1="-3.05" x2="-1.54" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-1.54" y1="0.75" x2="-3.49" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-5.74" y1="7.575" x2="-5.74" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="7.05" x2="-6.09" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="7.05" x2="-6.09" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-4.89" y1="7.55" x2="-4.89" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.84" y1="11.85" x2="-4.84" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-4.84" y1="9.35" x2="-3.14" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-3.14" y1="9.35" x2="-1.94" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.94" y1="9.35" x2="-0.24" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-0.24" y1="9.35" x2="-0.24" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-0.24" y1="11.85" x2="-4.84" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-3.89" y1="11.35" x2="-3.89" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-3.89" y1="9.85" x2="-1.19" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-1.19" y1="9.85" x2="-1.19" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-1.19" y1="11.35" x2="-3.89" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-3.14" y1="9.35" x2="-3.14" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-3.14" y1="6.7" x2="-1.94" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-1.94" y1="6.7" x2="-1.94" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="7.575" x2="-3.165" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-5.09" y1="7.55" x2="-5.09" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.69" y1="2.65" x2="-0.61" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.61" y1="2.65" x2="-0.06" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.06" y1="2.65" x2="5.02" y2="2.65" width="0.2032" layer="21"/>
<wire x1="5.02" y1="2.65" x2="10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="10.1" y1="-6.2" x2="11.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="11.85" y1="-6.2" x2="11.85" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="11.85" y1="-4.35" x2="11.85" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="11.85" y1="-3.65" x2="11.85" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.85" y1="-0.55" x2="11.85" y2="6.05" width="0.2032" layer="21"/>
<wire x1="10.15" y1="-3.65" x2="11.85" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="1.59" y1="0.75" x2="1.59" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="1.59" y1="-3.05" x2="3.54" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="3.54" y1="-3.05" x2="3.54" y2="0.75" width="0.2032" layer="51"/>
<wire x1="3.54" y1="0.75" x2="1.59" y2="0.75" width="0.2032" layer="51"/>
<wire x1="11.5" y1="6.05" x2="11.85" y2="6.05" width="0.2032" layer="21"/>
<wire x1="11.85" y1="-0.55" x2="11.5" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="11.5" y1="-0.55" x2="11.5" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.5" y1="-2.35" x2="11.85" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="11.85" y1="-4.35" x2="11.5" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="11.5" y1="-4.35" x2="11.5" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="11.5" y1="-5.35" x2="11.85" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="11.85" y1="7.05" x2="11.85" y2="7.55" width="0.2032" layer="21"/>
<wire x1="0.24" y1="11.85" x2="0.24" y2="9.35" width="0.2032" layer="21"/>
<wire x1="0.24" y1="9.35" x2="1.94" y2="9.35" width="0.2032" layer="21"/>
<wire x1="1.94" y1="9.35" x2="3.14" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.14" y1="9.35" x2="4.84" y2="9.35" width="0.2032" layer="21"/>
<wire x1="4.84" y1="9.35" x2="4.84" y2="11.85" width="0.2032" layer="21"/>
<wire x1="4.84" y1="11.85" x2="0.24" y2="11.85" width="0.2032" layer="21"/>
<wire x1="10.1" y1="7.55" x2="10.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="10.1" y1="2.65" x2="10.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="11.85" y1="7.05" x2="11.5" y2="7.05" width="0.2032" layer="21"/>
<wire x1="11.5" y1="7.05" x2="11.5" y2="6.05" width="0.2032" layer="21"/>
<wire x1="1.19" y1="11.35" x2="1.19" y2="9.85" width="0.2032" layer="21"/>
<wire x1="1.19" y1="9.85" x2="3.89" y2="9.85" width="0.2032" layer="21"/>
<wire x1="3.89" y1="9.85" x2="3.89" y2="11.35" width="0.2032" layer="21"/>
<wire x1="3.89" y1="11.35" x2="1.19" y2="11.35" width="0.2032" layer="21"/>
<wire x1="1.94" y1="9.35" x2="1.94" y2="6.7" width="0.2032" layer="21"/>
<wire x1="1.94" y1="6.7" x2="3.14" y2="6.7" width="0.2032" layer="21"/>
<wire x1="3.14" y1="6.7" x2="3.14" y2="9.35" width="0.2032" layer="21"/>
<wire x1="10.3" y1="7.55" x2="10.3" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="10.1" y1="2.65" x2="11.8" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.06" y1="7.55" x2="-0.06" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.06" y1="2.65" x2="-0.06" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="0.14" y1="7.55" x2="0.14" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-1.915" y1="7.575" x2="1.919" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-0.61" y1="2.65" x2="5.02" y2="2.65" width="0.2032" layer="21"/>
<wire x1="6.67" y1="0.75" x2="6.67" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="6.67" y1="-3.05" x2="8.62" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="8.62" y1="-3.05" x2="8.62" y2="0.75" width="0.2032" layer="51"/>
<wire x1="8.62" y1="0.75" x2="6.67" y2="0.75" width="0.2032" layer="51"/>
<wire x1="5.32" y1="11.85" x2="5.32" y2="9.35" width="0.2032" layer="21"/>
<wire x1="5.32" y1="9.35" x2="7.02" y2="9.35" width="0.2032" layer="21"/>
<wire x1="7.02" y1="9.35" x2="8.22" y2="9.35" width="0.2032" layer="21"/>
<wire x1="8.22" y1="9.35" x2="9.92" y2="9.35" width="0.2032" layer="21"/>
<wire x1="9.92" y1="9.35" x2="9.92" y2="11.85" width="0.2032" layer="21"/>
<wire x1="9.92" y1="11.85" x2="5.32" y2="11.85" width="0.2032" layer="21"/>
<wire x1="6.27" y1="11.35" x2="6.27" y2="9.85" width="0.2032" layer="21"/>
<wire x1="6.27" y1="9.85" x2="8.97" y2="9.85" width="0.2032" layer="21"/>
<wire x1="8.97" y1="9.85" x2="8.97" y2="11.35" width="0.2032" layer="21"/>
<wire x1="8.97" y1="11.35" x2="6.27" y2="11.35" width="0.2032" layer="21"/>
<wire x1="7.02" y1="9.35" x2="7.02" y2="6.7" width="0.2032" layer="21"/>
<wire x1="7.02" y1="6.7" x2="8.22" y2="6.7" width="0.2032" layer="21"/>
<wire x1="8.22" y1="6.7" x2="8.22" y2="9.35" width="0.2032" layer="21"/>
<wire x1="5.02" y1="7.55" x2="5.02" y2="2.65" width="0.2032" layer="21"/>
<wire x1="5.02" y1="2.65" x2="5.02" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="5.22" y1="7.55" x2="5.22" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="3.165" y1="7.575" x2="7.0104" y2="7.575" width="0.2032" layer="21"/>
<wire x1="8.2296" y1="7.575" x2="11.825" y2="7.575" width="0.2032" layer="21"/>
<pad name="A1" x="-2.54" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B1" x="-2.54" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A2" x="2.54" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B2" x="2.54" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A3" x="7.62" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B3" x="7.62" y="-5" drill="1.2" shape="long" rot="R90"/>
<text x="-2.79" y="1.35" size="0.8128" layer="21">1</text>
<text x="-6.575" y="-5.855" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="14.1" y="-5.75" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
</package>
<package name="255-402-5" urn="urn:adsk.eagle:footprint:10868/1" library_version="1">
<description>&lt;b&gt;Grid 5.0 mm&lt;/b&gt;</description>
<wire x1="-6.05" y1="6.05" x2="-5.7" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="6.05" x2="-5.7" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-0.55" x2="-6.05" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="-0.55" x2="-6.05" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="-2.35" x2="-5.7" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-2.35" x2="-5.7" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-4.35" x2="-6.05" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="-4.35" x2="-6.05" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="-5.35" x2="-5.7" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-5.35" x2="-5.7" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-6.2" x2="-5.05" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.05" y1="-6.2" x2="-4.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.85" y1="-6.2" x2="4.9" y2="-6.2" width="0.2032" layer="51"/>
<wire x1="4.9" y1="-6.2" x2="6.65" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="6.65" y1="-6.2" x2="6.65" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="6.65" y1="-4.35" x2="6.65" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="6.65" y1="-3.65" x2="6.65" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="6.65" y1="-0.55" x2="6.65" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="-3.65" x2="-5" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-5" y1="-3.65" x2="4.9" y2="-3.65" width="0.2032" layer="51"/>
<wire x1="4.95" y1="-3.65" x2="6.65" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-3.45" y1="0.75" x2="-3.45" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-3.45" y1="-3.05" x2="-1.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="-3.05" x2="-1.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="0.75" x2="-3.45" y2="0.75" width="0.2032" layer="51"/>
<wire x1="6.3" y1="6.05" x2="6.65" y2="6.05" width="0.2032" layer="21"/>
<wire x1="6.65" y1="-0.55" x2="6.3" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="6.3" y1="-0.55" x2="6.3" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="6.3" y1="-2.35" x2="6.65" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="6.65" y1="-4.35" x2="6.3" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="6.3" y1="-4.35" x2="6.3" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="6.3" y1="-5.35" x2="6.65" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="1.5" y1="0.75" x2="1.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="1.5" y1="-3.05" x2="3.5" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="3.5" y1="-3.05" x2="3.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="3.5" y1="0.75" x2="1.5" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-5.7" y1="7.575" x2="-5.7" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="7.05" x2="-6.05" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-6.05" y1="7.05" x2="-6.05" y2="6.05" width="0.2032" layer="21"/>
<wire x1="6.65" y1="7.05" x2="6.65" y2="7.55" width="0.2032" layer="21"/>
<wire x1="-4.85" y1="7.55" x2="-4.85" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.8" y1="11.85" x2="-4.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-4.8" y1="9.35" x2="-3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="9.35" x2="-1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.9" y1="9.35" x2="-0.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-0.2" y1="9.35" x2="-0.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-0.2" y1="11.85" x2="-4.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="7.55" x2="-0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.1" y1="2.65" x2="-0.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="4.9" y1="7.55" x2="4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="4.9" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="6.65" y1="7.05" x2="6.3" y2="7.05" width="0.2032" layer="21"/>
<wire x1="6.3" y1="7.05" x2="6.3" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-3.85" y1="11.35" x2="-3.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-3.85" y1="9.85" x2="-1.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-1.15" y1="9.85" x2="-1.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-1.15" y1="11.35" x2="-3.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="9.35" x2="-3.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-3.1" y1="6.7" x2="-1.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-1.9" y1="6.7" x2="-1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-5.7" y1="7.575" x2="-3.125" y2="7.575" width="0.2032" layer="21"/>
<wire x1="0.2" y1="11.85" x2="0.2" y2="9.35" width="0.2032" layer="21"/>
<wire x1="0.2" y1="9.35" x2="1.9" y2="9.35" width="0.2032" layer="21"/>
<wire x1="1.9" y1="9.35" x2="3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.1" y1="9.35" x2="4.8" y2="9.35" width="0.2032" layer="21"/>
<wire x1="4.8" y1="9.35" x2="4.8" y2="11.85" width="0.2032" layer="21"/>
<wire x1="4.8" y1="11.85" x2="0.2" y2="11.85" width="0.2032" layer="21"/>
<wire x1="1.15" y1="11.35" x2="1.15" y2="9.85" width="0.2032" layer="21"/>
<wire x1="1.15" y1="9.85" x2="3.85" y2="9.85" width="0.2032" layer="21"/>
<wire x1="3.85" y1="9.85" x2="3.85" y2="11.35" width="0.2032" layer="21"/>
<wire x1="3.85" y1="11.35" x2="1.15" y2="11.35" width="0.2032" layer="21"/>
<wire x1="1.9" y1="9.35" x2="1.9" y2="6.7" width="0.2032" layer="21"/>
<wire x1="1.9" y1="6.7" x2="3.1" y2="6.7" width="0.2032" layer="21"/>
<wire x1="3.1" y1="6.7" x2="3.1" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.825" y1="7.575" x2="1.875" y2="7.575" width="0.2032" layer="21"/>
<wire x1="3.125" y1="7.575" x2="6.625" y2="7.575" width="0.2032" layer="21"/>
<wire x1="0.1" y1="7.55" x2="0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="0.1" y1="2.65" x2="0.1" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.05" y1="7.55" x2="-5.05" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.65" y1="2.65" x2="-0.1" y2="2.65" width="0.2032" layer="21"/>
<wire x1="0.1" y1="2.65" x2="4.9" y2="2.65" width="0.2032" layer="21"/>
<wire x1="4.9" y1="2.65" x2="6.6" y2="2.65" width="0.2032" layer="21"/>
<pad name="A1" x="-2.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B1" x="-2.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A2" x="2.5" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B2" x="2.5" y="-5" drill="1.2" shape="long" rot="R90"/>
<text x="-2.75" y="1.35" size="0.8128" layer="21">1</text>
<text x="-6.535" y="-5.855" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="8.9" y="-5.75" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
</package>
<package name="255-402-508" urn="urn:adsk.eagle:footprint:10869/1" library_version="1">
<description>&lt;b&gt;Grid 5.0 mm&lt;/b&gt;</description>
<wire x1="-6.09" y1="6.05" x2="-5.74" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="6.05" x2="-5.74" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-0.55" x2="-6.09" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="-0.55" x2="-6.09" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="-2.35" x2="-5.74" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-2.35" x2="-5.74" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-4.35" x2="-6.09" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="-4.35" x2="-6.09" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="-5.35" x2="-5.74" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-5.35" x2="-5.74" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="-6.2" x2="-5.09" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.09" y1="-6.2" x2="-4.89" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.89" y1="-6.2" x2="5.02" y2="-6.2" width="0.2032" layer="51"/>
<wire x1="-5.74" y1="-3.65" x2="-5.04" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="-5.04" y1="-3.65" x2="5.02" y2="-3.65" width="0.2032" layer="51"/>
<wire x1="-3.49" y1="0.75" x2="-3.49" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-3.49" y1="-3.05" x2="-1.54" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="-1.54" y1="-3.05" x2="-1.54" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-1.54" y1="0.75" x2="-3.49" y2="0.75" width="0.2032" layer="51"/>
<wire x1="-5.74" y1="7.575" x2="-5.74" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="7.05" x2="-6.09" y2="7.05" width="0.2032" layer="21"/>
<wire x1="-6.09" y1="7.05" x2="-6.09" y2="6.05" width="0.2032" layer="21"/>
<wire x1="-4.89" y1="7.55" x2="-4.89" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-4.84" y1="11.85" x2="-4.84" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-4.84" y1="9.35" x2="-3.14" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-3.14" y1="9.35" x2="-1.94" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-1.94" y1="9.35" x2="-0.24" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-0.24" y1="9.35" x2="-0.24" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-0.24" y1="11.85" x2="-4.84" y2="11.85" width="0.2032" layer="21"/>
<wire x1="-3.89" y1="11.35" x2="-3.89" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-3.89" y1="9.85" x2="-1.19" y2="9.85" width="0.2032" layer="21"/>
<wire x1="-1.19" y1="9.85" x2="-1.19" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-1.19" y1="11.35" x2="-3.89" y2="11.35" width="0.2032" layer="21"/>
<wire x1="-3.14" y1="9.35" x2="-3.14" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-3.14" y1="6.7" x2="-1.94" y2="6.7" width="0.2032" layer="21"/>
<wire x1="-1.94" y1="6.7" x2="-1.94" y2="9.35" width="0.2032" layer="21"/>
<wire x1="-5.74" y1="7.575" x2="-3.165" y2="7.575" width="0.2032" layer="21"/>
<wire x1="-5.09" y1="7.55" x2="-5.09" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-5.69" y1="2.65" x2="-0.06" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.06" y1="2.65" x2="5.02" y2="2.65" width="0.2032" layer="21"/>
<wire x1="5.02" y1="-6.2" x2="6.77" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="6.77" y1="-6.2" x2="6.77" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="6.77" y1="-4.35" x2="6.77" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="6.77" y1="-3.65" x2="6.77" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="6.77" y1="-0.55" x2="6.77" y2="6.05" width="0.2032" layer="21"/>
<wire x1="5.07" y1="-3.65" x2="6.77" y2="-3.65" width="0.2032" layer="21"/>
<wire x1="1.59" y1="0.75" x2="1.59" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="1.59" y1="-3.05" x2="3.54" y2="-3.05" width="0.2032" layer="51"/>
<wire x1="3.54" y1="-3.05" x2="3.54" y2="0.75" width="0.2032" layer="51"/>
<wire x1="3.54" y1="0.75" x2="1.59" y2="0.75" width="0.2032" layer="51"/>
<wire x1="6.42" y1="6.05" x2="6.77" y2="6.05" width="0.2032" layer="21"/>
<wire x1="6.77" y1="-0.55" x2="6.42" y2="-0.55" width="0.2032" layer="21"/>
<wire x1="6.42" y1="-0.55" x2="6.42" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="6.42" y1="-2.35" x2="6.77" y2="-2.35" width="0.2032" layer="21"/>
<wire x1="6.77" y1="-4.35" x2="6.42" y2="-4.35" width="0.2032" layer="21"/>
<wire x1="6.42" y1="-4.35" x2="6.42" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="6.42" y1="-5.35" x2="6.77" y2="-5.35" width="0.2032" layer="21"/>
<wire x1="6.77" y1="7.05" x2="6.77" y2="7.55" width="0.2032" layer="21"/>
<wire x1="0.24" y1="11.85" x2="0.24" y2="9.35" width="0.2032" layer="21"/>
<wire x1="0.24" y1="9.35" x2="1.94" y2="9.35" width="0.2032" layer="21"/>
<wire x1="1.94" y1="9.35" x2="3.14" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.14" y1="9.35" x2="4.84" y2="9.35" width="0.2032" layer="21"/>
<wire x1="4.84" y1="9.35" x2="4.84" y2="11.85" width="0.2032" layer="21"/>
<wire x1="4.84" y1="11.85" x2="0.24" y2="11.85" width="0.2032" layer="21"/>
<wire x1="5.02" y1="7.55" x2="5.02" y2="2.65" width="0.2032" layer="21"/>
<wire x1="5.02" y1="2.65" x2="5.02" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="6.77" y1="7.05" x2="6.42" y2="7.05" width="0.2032" layer="21"/>
<wire x1="6.42" y1="7.05" x2="6.42" y2="6.05" width="0.2032" layer="21"/>
<wire x1="1.19" y1="11.35" x2="1.19" y2="9.85" width="0.2032" layer="21"/>
<wire x1="1.19" y1="9.85" x2="3.89" y2="9.85" width="0.2032" layer="21"/>
<wire x1="3.89" y1="9.85" x2="3.89" y2="11.35" width="0.2032" layer="21"/>
<wire x1="3.89" y1="11.35" x2="1.19" y2="11.35" width="0.2032" layer="21"/>
<wire x1="1.94" y1="9.35" x2="1.94" y2="6.7" width="0.2032" layer="21"/>
<wire x1="1.94" y1="6.7" x2="3.14" y2="6.7" width="0.2032" layer="21"/>
<wire x1="3.14" y1="6.7" x2="3.14" y2="9.35" width="0.2032" layer="21"/>
<wire x1="3.165" y1="7.575" x2="6.745" y2="7.575" width="0.2032" layer="21"/>
<wire x1="5.22" y1="7.55" x2="5.22" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="5.02" y1="2.65" x2="6.72" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.06" y1="7.55" x2="-0.06" y2="2.65" width="0.2032" layer="21"/>
<wire x1="-0.06" y1="2.65" x2="-0.06" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="0.14" y1="7.55" x2="0.14" y2="-6.2" width="0.2032" layer="21"/>
<wire x1="-1.915" y1="7.575" x2="1.919" y2="7.575" width="0.2032" layer="21"/>
<pad name="A1" x="-2.54" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B1" x="-2.54" y="-5" drill="1.2" shape="long" rot="R90"/>
<pad name="A2" x="2.54" y="0" drill="1.2" shape="long" rot="R90"/>
<pad name="B2" x="2.54" y="-5" drill="1.2" shape="long" rot="R90"/>
<text x="-2.79" y="1.35" size="0.8128" layer="21">1</text>
<text x="-6.575" y="-5.855" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="9.02" y="-5.75" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="255-404-5" urn="urn:adsk.eagle:package:10950/1" type="box" library_version="1">
<description>Grid 5.0 mm</description>
<packageinstances>
<packageinstance name="255-404-5"/>
</packageinstances>
</package3d>
<package3d name="255-408-5" urn="urn:adsk.eagle:package:10986/1" type="box" library_version="1">
<description>Grid 5.0 mm</description>
<packageinstances>
<packageinstance name="255-408-5"/>
</packageinstances>
</package3d>
<package3d name="255-403-5" urn="urn:adsk.eagle:package:10951/1" type="box" library_version="1">
<description>Grid 5.0 mm</description>
<packageinstances>
<packageinstance name="255-403-5"/>
</packageinstances>
</package3d>
<package3d name="255-403-508" urn="urn:adsk.eagle:package:10957/1" type="box" library_version="1">
<description>Grid 5.0 mm</description>
<packageinstances>
<packageinstance name="255-403-508"/>
</packageinstances>
</package3d>
<package3d name="255-402-5" urn="urn:adsk.eagle:package:10944/1" type="box" library_version="1">
<description>Grid 5.0 mm</description>
<packageinstances>
<packageinstance name="255-402-5"/>
</packageinstances>
</package3d>
<package3d name="255-402-508" urn="urn:adsk.eagle:package:10959/1" type="box" library_version="1">
<description>Grid 5.0 mm</description>
<packageinstances>
<packageinstance name="255-402-508"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="KL-4" urn="urn:adsk.eagle:symbol:10789/1" library_version="1">
<wire x1="-2.54" y1="2.794" x2="-2.54" y2="2.286" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.286" x2="-1.016" y2="2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.286" x2="-1.016" y2="2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.794" x2="-2.54" y2="2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0.254" x2="-2.54" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-0.254" x2="-1.016" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-0.254" x2="-1.016" y2="0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0.254" x2="-2.54" y2="0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.286" x2="-2.54" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.794" x2="-1.016" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.794" x2="-1.016" y2="-2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.286" x2="-2.54" y2="-2.286" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-4.826" x2="-2.54" y2="-5.334" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.334" x2="-1.016" y2="-5.334" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-5.334" x2="-1.016" y2="-4.826" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-4.826" x2="-2.54" y2="-4.826" width="0.254" layer="94"/>
<text x="-5.08" y="5.08" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-8.89" size="1.778" layer="96">&gt;VALUE</text>
<pin name="-1" x="-7.62" y="2.54" visible="pin" length="short" direction="pas"/>
<pin name="-2" x="-7.62" y="0" visible="pin" length="short" direction="pas"/>
<pin name="-3" x="-7.62" y="-2.54" visible="pin" length="short" direction="pas"/>
<pin name="-4" x="-7.62" y="-5.08" visible="pin" length="short" direction="pas"/>
<pin name="B-4" x="-5.08" y="-5.08" visible="off" length="short" direction="pas"/>
<pin name="B-1" x="-5.08" y="2.54" visible="off" length="short" direction="pas"/>
<pin name="B-2" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
<pin name="B-3" x="-5.08" y="-2.54" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="KL-8" urn="urn:adsk.eagle:symbol:10782/1" library_version="1">
<wire x1="-2.54" y1="7.874" x2="-2.54" y2="7.366" width="0.254" layer="94"/>
<wire x1="-2.54" y1="7.366" x2="-1.016" y2="7.366" width="0.254" layer="94"/>
<wire x1="-1.016" y1="7.366" x2="-1.016" y2="7.874" width="0.254" layer="94"/>
<wire x1="-1.016" y1="7.874" x2="-2.54" y2="7.874" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.334" x2="-2.54" y2="4.826" width="0.254" layer="94"/>
<wire x1="-2.54" y1="4.826" x2="-1.016" y2="4.826" width="0.254" layer="94"/>
<wire x1="-1.016" y1="4.826" x2="-1.016" y2="5.334" width="0.254" layer="94"/>
<wire x1="-1.016" y1="5.334" x2="-2.54" y2="5.334" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.794" x2="-2.54" y2="2.286" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.286" x2="-1.016" y2="2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.286" x2="-1.016" y2="2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.794" x2="-2.54" y2="2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0.254" x2="-2.54" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-0.254" x2="-1.016" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-0.254" x2="-1.016" y2="0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0.254" x2="-2.54" y2="0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.286" x2="-2.54" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.794" x2="-1.016" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.794" x2="-1.016" y2="-2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.286" x2="-2.54" y2="-2.286" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-4.826" x2="-2.54" y2="-5.334" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.334" x2="-1.016" y2="-5.334" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-5.334" x2="-1.016" y2="-4.826" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-4.826" x2="-2.54" y2="-4.826" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.366" x2="-2.54" y2="-7.874" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.874" x2="-1.016" y2="-7.874" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-7.874" x2="-1.016" y2="-7.366" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-7.366" x2="-2.54" y2="-7.366" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-9.906" x2="-2.54" y2="-10.414" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-10.414" x2="-1.016" y2="-10.414" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-10.414" x2="-1.016" y2="-9.906" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-9.906" x2="-2.54" y2="-9.906" width="0.254" layer="94"/>
<text x="-5.08" y="10.16" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-13.97" size="1.778" layer="96">&gt;VALUE</text>
<pin name="-1" x="-7.62" y="7.62" visible="pin" length="short" direction="pas"/>
<pin name="-2" x="-7.62" y="5.08" visible="pin" length="short" direction="pas"/>
<pin name="-3" x="-7.62" y="2.54" visible="pin" length="short" direction="pas"/>
<pin name="-4" x="-7.62" y="0" visible="pin" length="short" direction="pas"/>
<pin name="-5" x="-7.62" y="-2.54" visible="pin" length="short" direction="pas"/>
<pin name="-6" x="-7.62" y="-5.08" visible="pin" length="short" direction="pas"/>
<pin name="-7" x="-7.62" y="-7.62" visible="pin" length="short" direction="pas"/>
<pin name="-8" x="-7.62" y="-10.16" visible="pin" length="short" direction="pas"/>
<pin name="B-8" x="-5.08" y="-10.16" visible="off" length="short" direction="pas"/>
<pin name="B-1" x="-5.08" y="7.62" visible="off" length="short" direction="pas"/>
<pin name="B-2" x="-5.08" y="5.08" visible="off" length="short" direction="pas"/>
<pin name="B-3" x="-5.08" y="2.54" visible="off" length="short" direction="pas"/>
<pin name="B-4" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
<pin name="B-5" x="-5.08" y="-2.54" visible="off" length="short" direction="pas"/>
<pin name="B-6" x="-5.08" y="-5.08" visible="off" length="short" direction="pas"/>
<pin name="B-7" x="-5.08" y="-7.62" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="KL-3" urn="urn:adsk.eagle:symbol:10792/1" library_version="1">
<wire x1="-2.54" y1="2.794" x2="-2.54" y2="2.286" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.286" x2="-1.016" y2="2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.286" x2="-1.016" y2="2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="2.794" x2="-2.54" y2="2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0.254" x2="-2.54" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-0.254" x2="-1.016" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-0.254" x2="-1.016" y2="0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0.254" x2="-2.54" y2="0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.286" x2="-2.54" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.794" x2="-1.016" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.794" x2="-1.016" y2="-2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.286" x2="-2.54" y2="-2.286" width="0.254" layer="94"/>
<text x="-5.08" y="5.08" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-6.35" size="1.778" layer="96">&gt;VALUE</text>
<pin name="-1" x="-7.62" y="2.54" visible="pin" length="short" direction="pas"/>
<pin name="-2" x="-7.62" y="0" visible="pin" length="short" direction="pas"/>
<pin name="-3" x="-7.62" y="-2.54" visible="pin" length="short" direction="pas"/>
<pin name="B-3" x="-5.08" y="-2.54" visible="off" length="short" direction="pas"/>
<pin name="B-1" x="-5.08" y="2.54" visible="off" length="short" direction="pas"/>
<pin name="B-2" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="KL-2" urn="urn:adsk.eagle:symbol:10794/1" library_version="1">
<wire x1="-2.54" y1="0.254" x2="-2.54" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-0.254" x2="-1.016" y2="-0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-0.254" x2="-1.016" y2="0.254" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0.254" x2="-2.54" y2="0.254" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.286" x2="-2.54" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-2.794" x2="-1.016" y2="-2.794" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.794" x2="-1.016" y2="-2.286" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-2.286" x2="-2.54" y2="-2.286" width="0.254" layer="94"/>
<text x="-5.08" y="2.54" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-6.35" size="1.778" layer="96">&gt;VALUE</text>
<pin name="-1" x="-7.62" y="0" visible="pin" length="short" direction="pas"/>
<pin name="-2" x="-7.62" y="-2.54" visible="pin" length="short" direction="pas"/>
<pin name="B-2" x="-5.08" y="-2.54" visible="off" length="short" direction="pas"/>
<pin name="B-1" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="255-404-5" urn="urn:adsk.eagle:component:11056/1" prefix="X" library_version="1">
<description>&lt;b&gt;WAGO&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="KL-4" x="0" y="0"/>
</gates>
<devices>
<device name="" package="255-404-5">
<connects>
<connect gate="G$1" pin="-1" pad="A1"/>
<connect gate="G$1" pin="-2" pad="A2"/>
<connect gate="G$1" pin="-3" pad="A3"/>
<connect gate="G$1" pin="-4" pad="A4"/>
<connect gate="G$1" pin="B-1" pad="B1"/>
<connect gate="G$1" pin="B-2" pad="B2"/>
<connect gate="G$1" pin="B-3" pad="B3"/>
<connect gate="G$1" pin="B-4" pad="B4"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10950/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="255-408-5" urn="urn:adsk.eagle:component:11080/1" prefix="X" library_version="1">
<description>&lt;b&gt;WAGO&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="KL-8" x="0" y="0"/>
</gates>
<devices>
<device name="" package="255-408-5">
<connects>
<connect gate="G$1" pin="-1" pad="A1"/>
<connect gate="G$1" pin="-2" pad="A2"/>
<connect gate="G$1" pin="-3" pad="A3"/>
<connect gate="G$1" pin="-4" pad="A4"/>
<connect gate="G$1" pin="-5" pad="A5"/>
<connect gate="G$1" pin="-6" pad="A6"/>
<connect gate="G$1" pin="-7" pad="A7"/>
<connect gate="G$1" pin="-8" pad="A8"/>
<connect gate="G$1" pin="B-1" pad="B1"/>
<connect gate="G$1" pin="B-2" pad="B2"/>
<connect gate="G$1" pin="B-3" pad="B3"/>
<connect gate="G$1" pin="B-4" pad="B4"/>
<connect gate="G$1" pin="B-5" pad="B5"/>
<connect gate="G$1" pin="B-6" pad="B6"/>
<connect gate="G$1" pin="B-7" pad="B7"/>
<connect gate="G$1" pin="B-8" pad="B8"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10986/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="255-403-5" urn="urn:adsk.eagle:component:11062/1" prefix="X" library_version="1">
<description>&lt;b&gt;WAGO&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="KL-3" x="0" y="0"/>
</gates>
<devices>
<device name="" package="255-403-5">
<connects>
<connect gate="G$1" pin="-1" pad="A1"/>
<connect gate="G$1" pin="-2" pad="A2"/>
<connect gate="G$1" pin="-3" pad="A3"/>
<connect gate="G$1" pin="B-1" pad="B1"/>
<connect gate="G$1" pin="B-2" pad="B2"/>
<connect gate="G$1" pin="B-3" pad="B3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10951/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
<device name="08" package="255-403-508">
<connects>
<connect gate="G$1" pin="-1" pad="A1"/>
<connect gate="G$1" pin="-2" pad="A2"/>
<connect gate="G$1" pin="-3" pad="A3"/>
<connect gate="G$1" pin="B-1" pad="B1"/>
<connect gate="G$1" pin="B-2" pad="B2"/>
<connect gate="G$1" pin="B-3" pad="B3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10957/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="255-402-5" urn="urn:adsk.eagle:component:11055/1" prefix="X" uservalue="yes" library_version="1">
<description>&lt;b&gt;WAGO&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="KL-2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="255-402-5">
<connects>
<connect gate="G$1" pin="-1" pad="A1"/>
<connect gate="G$1" pin="-2" pad="A2"/>
<connect gate="G$1" pin="B-1" pad="B1"/>
<connect gate="G$1" pin="B-2" pad="B2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10944/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
<device name="08" package="255-402-508">
<connects>
<connect gate="G$1" pin="-1" pad="A1"/>
<connect gate="G$1" pin="-2" pad="A2"/>
<connect gate="G$1" pin="B-1" pad="B1"/>
<connect gate="G$1" pin="B-2" pad="B2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10959/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
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
<part name="U$1" library="ESP32-DEVKITV1" deviceset="ESP32DEVKITV1" device=""/>
<part name="X1" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-404-5" device="" package3d_urn="urn:adsk.eagle:package:10950/1"/>
<part name="X2" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-404-5" device="" package3d_urn="urn:adsk.eagle:package:10950/1"/>
<part name="X3" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-408-5" device="" package3d_urn="urn:adsk.eagle:package:10986/1"/>
<part name="X4" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-403-5" device="08" package3d_urn="urn:adsk.eagle:package:10957/1"/>
<part name="X5" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-403-5" device="08" package3d_urn="urn:adsk.eagle:package:10957/1"/>
<part name="X7" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-403-5" device="08" package3d_urn="urn:adsk.eagle:package:10957/1"/>
<part name="X8" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-403-5" device="08" package3d_urn="urn:adsk.eagle:package:10957/1"/>
<part name="X9" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-403-5" device="08" package3d_urn="urn:adsk.eagle:package:10957/1"/>
<part name="X10" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-402-5" device="08" package3d_urn="urn:adsk.eagle:package:10959/1"/>
<part name="X11" library="con-wago255" library_urn="urn:adsk.eagle:library:198" deviceset="255-404-5" device="" package3d_urn="urn:adsk.eagle:package:10950/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="53.34" y="43.18" smashed="yes" rot="R90">
<attribute name="NAME" x="48.26" y="16.51" size="1.27" layer="95" rot="R180"/>
</instance>
<instance part="X1" gate="G$1" x="104.14" y="58.42" smashed="yes">
<attribute name="NAME" x="99.06" y="63.5" size="1.778" layer="95"/>
<attribute name="VALUE" x="99.06" y="49.53" size="1.778" layer="96"/>
</instance>
<instance part="X2" gate="G$1" x="104.14" y="38.1" smashed="yes">
<attribute name="NAME" x="99.06" y="43.18" size="1.778" layer="95"/>
<attribute name="VALUE" x="99.06" y="29.21" size="1.778" layer="96"/>
</instance>
<instance part="X3" gate="G$1" x="-5.08" y="48.26" smashed="yes" rot="R180">
<attribute name="NAME" x="0" y="38.1" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="0" y="62.23" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="X4" gate="G$1" x="104.14" y="17.78" smashed="yes">
<attribute name="NAME" x="99.06" y="22.86" size="1.778" layer="95"/>
<attribute name="VALUE" x="99.06" y="11.43" size="1.778" layer="96"/>
</instance>
<instance part="X5" gate="G$1" x="104.14" y="2.54" smashed="yes">
<attribute name="NAME" x="99.06" y="7.62" size="1.778" layer="95"/>
<attribute name="VALUE" x="99.06" y="-3.81" size="1.778" layer="96"/>
</instance>
<instance part="X7" gate="G$1" x="-5.08" y="17.78" smashed="yes" rot="R180">
<attribute name="NAME" x="0" y="12.7" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="0" y="24.13" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="X8" gate="G$1" x="-5.08" y="5.08" smashed="yes" rot="R180">
<attribute name="NAME" x="0" y="0" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="0" y="11.43" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="X9" gate="G$1" x="-5.08" y="-12.7" smashed="yes" rot="R180">
<attribute name="NAME" x="0" y="-17.78" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="0" y="-6.35" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="X10" gate="G$1" x="-5.08" y="66.04" smashed="yes" rot="R180">
<attribute name="NAME" x="0" y="63.5" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="0" y="72.39" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="X11" gate="G$1" x="104.14" y="73.66" smashed="yes">
<attribute name="NAME" x="99.06" y="78.74" size="1.778" layer="95"/>
<attribute name="VALUE" x="99.06" y="64.77" size="1.778" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="GND1"/>
<wire x1="35.56" y1="22.86" x2="25.4" y2="22.86" width="0.1524" layer="91"/>
<pinref part="X10" gate="G$1" pin="-2"/>
<wire x1="25.4" y1="22.86" x2="25.4" y2="53.34" width="0.1524" layer="91"/>
<wire x1="25.4" y1="53.34" x2="25.4" y2="68.58" width="0.1524" layer="91"/>
<wire x1="25.4" y1="68.58" x2="2.54" y2="68.58" width="0.1524" layer="91"/>
<pinref part="X3" gate="G$1" pin="-6"/>
<wire x1="2.54" y1="53.34" x2="25.4" y2="53.34" width="0.1524" layer="91"/>
<junction x="25.4" y="53.34"/>
<pinref part="X7" gate="G$1" pin="-1"/>
<wire x1="2.54" y1="15.24" x2="25.4" y2="15.24" width="0.1524" layer="91"/>
<wire x1="25.4" y1="15.24" x2="25.4" y2="22.86" width="0.1524" layer="91"/>
<junction x="25.4" y="22.86"/>
<pinref part="X8" gate="G$1" pin="-1"/>
<wire x1="2.54" y1="2.54" x2="25.4" y2="2.54" width="0.1524" layer="91"/>
<wire x1="25.4" y1="2.54" x2="25.4" y2="15.24" width="0.1524" layer="91"/>
<junction x="25.4" y="15.24"/>
<pinref part="X9" gate="G$1" pin="-1"/>
<wire x1="2.54" y1="-15.24" x2="25.4" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="25.4" y1="-15.24" x2="25.4" y2="2.54" width="0.1524" layer="91"/>
<junction x="25.4" y="2.54"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="X5" gate="G$1" pin="-1"/>
<wire x1="96.52" y1="5.08" x2="81.28" y2="5.08" width="0.1524" layer="91"/>
<wire x1="81.28" y1="5.08" x2="81.28" y2="20.32" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="GND"/>
<wire x1="81.28" y1="20.32" x2="81.28" y2="22.86" width="0.1524" layer="91"/>
<wire x1="81.28" y1="22.86" x2="71.12" y2="22.86" width="0.1524" layer="91"/>
<pinref part="X4" gate="G$1" pin="-1"/>
<wire x1="96.52" y1="20.32" x2="81.28" y2="20.32" width="0.1524" layer="91"/>
<junction x="81.28" y="20.32"/>
<pinref part="X2" gate="G$1" pin="-3"/>
<wire x1="96.52" y1="35.56" x2="81.28" y2="35.56" width="0.1524" layer="91"/>
<wire x1="81.28" y1="35.56" x2="81.28" y2="22.86" width="0.1524" layer="91"/>
<junction x="81.28" y="22.86"/>
<pinref part="X1" gate="G$1" pin="-4"/>
<wire x1="96.52" y1="53.34" x2="81.28" y2="53.34" width="0.1524" layer="91"/>
<wire x1="81.28" y1="53.34" x2="81.28" y2="35.56" width="0.1524" layer="91"/>
<junction x="81.28" y="35.56"/>
<pinref part="X11" gate="G$1" pin="-4"/>
<wire x1="96.52" y1="68.58" x2="81.28" y2="68.58" width="0.1524" layer="91"/>
<wire x1="81.28" y1="68.58" x2="81.28" y2="53.34" width="0.1524" layer="91"/>
<junction x="81.28" y="53.34"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="IO1"/>
<wire x1="71.12" y1="50.8" x2="76.2" y2="50.8" width="0.1524" layer="91"/>
<wire x1="76.2" y1="50.8" x2="76.2" y2="60.96" width="0.1524" layer="91"/>
<pinref part="X1" gate="G$1" pin="-1"/>
<wire x1="76.2" y1="60.96" x2="96.52" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="X1" gate="G$1" pin="-2"/>
<wire x1="96.52" y1="58.42" x2="78.74" y2="58.42" width="0.1524" layer="91"/>
<wire x1="78.74" y1="58.42" x2="78.74" y2="48.26" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO3"/>
<wire x1="78.74" y1="48.26" x2="71.12" y2="48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="X1" gate="G$1" pin="-3"/>
<wire x1="96.52" y1="55.88" x2="86.36" y2="55.88" width="0.1524" layer="91"/>
<wire x1="86.36" y1="55.88" x2="86.36" y2="10.16" width="0.1524" layer="91"/>
<wire x1="86.36" y1="10.16" x2="33.02" y2="10.16" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="VIN"/>
<wire x1="33.02" y1="10.16" x2="33.02" y2="20.32" width="0.1524" layer="91"/>
<wire x1="33.02" y1="20.32" x2="35.56" y2="20.32" width="0.1524" layer="91"/>
<pinref part="X3" gate="G$1" pin="-8"/>
<wire x1="2.54" y1="58.42" x2="20.32" y2="58.42" width="0.1524" layer="91"/>
<wire x1="20.32" y1="58.42" x2="20.32" y2="20.32" width="0.1524" layer="91"/>
<wire x1="20.32" y1="20.32" x2="33.02" y2="20.32" width="0.1524" layer="91"/>
<junction x="33.02" y="20.32"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="X3" gate="G$1" pin="-7"/>
<wire x1="2.54" y1="55.88" x2="27.94" y2="55.88" width="0.1524" layer="91"/>
<wire x1="27.94" y1="55.88" x2="27.94" y2="33.02" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO27"/>
<wire x1="27.94" y1="33.02" x2="35.56" y2="33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="X3" gate="G$1" pin="-4"/>
<wire x1="2.54" y1="48.26" x2="15.24" y2="48.26" width="0.1524" layer="91"/>
<wire x1="15.24" y1="48.26" x2="15.24" y2="71.12" width="0.1524" layer="91"/>
<wire x1="15.24" y1="71.12" x2="73.66" y2="71.12" width="0.1524" layer="91"/>
<wire x1="73.66" y1="71.12" x2="73.66" y2="43.18" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO19"/>
<wire x1="73.66" y1="43.18" x2="71.12" y2="43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="X3" gate="G$1" pin="-3"/>
<wire x1="2.54" y1="45.72" x2="12.7" y2="45.72" width="0.1524" layer="91"/>
<wire x1="12.7" y1="45.72" x2="12.7" y2="73.66" width="0.1524" layer="91"/>
<wire x1="12.7" y1="73.66" x2="76.2" y2="73.66" width="0.1524" layer="91"/>
<wire x1="76.2" y1="73.66" x2="76.2" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO18"/>
<wire x1="76.2" y1="40.64" x2="71.12" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="X8" gate="G$1" pin="-2"/>
<wire x1="2.54" y1="5.08" x2="15.24" y2="5.08" width="0.1524" layer="91"/>
<wire x1="15.24" y1="5.08" x2="15.24" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO33"/>
<wire x1="15.24" y1="40.64" x2="35.56" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="X7" gate="G$1" pin="-2"/>
<wire x1="2.54" y1="17.78" x2="17.78" y2="17.78" width="0.1524" layer="91"/>
<wire x1="17.78" y1="17.78" x2="17.78" y2="38.1" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO25"/>
<wire x1="17.78" y1="38.1" x2="35.56" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="IO26"/>
<wire x1="35.56" y1="35.56" x2="22.86" y2="35.56" width="0.1524" layer="91"/>
<wire x1="22.86" y1="35.56" x2="22.86" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="X9" gate="G$1" pin="-2"/>
<wire x1="22.86" y1="-12.7" x2="2.54" y2="-12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="X3" gate="G$1" pin="-2"/>
<wire x1="2.54" y1="43.18" x2="12.7" y2="43.18" width="0.1524" layer="91"/>
<wire x1="12.7" y1="43.18" x2="12.7" y2="-7.62" width="0.1524" layer="91"/>
<wire x1="12.7" y1="-7.62" x2="76.2" y2="-7.62" width="0.1524" layer="91"/>
<wire x1="76.2" y1="-7.62" x2="76.2" y2="38.1" width="0.1524" layer="91"/>
<wire x1="76.2" y1="38.1" x2="78.74" y2="38.1" width="0.1524" layer="91"/>
<wire x1="78.74" y1="38.1" x2="78.74" y2="45.72" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO21"/>
<wire x1="78.74" y1="45.72" x2="71.12" y2="45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="IO5"/>
<wire x1="71.12" y1="38.1" x2="73.66" y2="38.1" width="0.1524" layer="91"/>
<wire x1="73.66" y1="38.1" x2="73.66" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="73.66" y1="-5.08" x2="7.62" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="7.62" y1="-5.08" x2="7.62" y2="40.64" width="0.1524" layer="91"/>
<pinref part="X3" gate="G$1" pin="-1"/>
<wire x1="7.62" y1="40.64" x2="2.54" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="X10" gate="G$1" pin="-1"/>
<wire x1="2.54" y1="66.04" x2="30.48" y2="66.04" width="0.1524" layer="91"/>
<wire x1="30.48" y1="66.04" x2="30.48" y2="27.94" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO12"/>
<wire x1="30.48" y1="27.94" x2="35.56" y2="27.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="X2" gate="G$1" pin="-1"/>
<wire x1="96.52" y1="40.64" x2="91.44" y2="40.64" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IO2"/>
<wire x1="91.44" y1="40.64" x2="91.44" y2="27.94" width="0.1524" layer="91"/>
<wire x1="91.44" y1="27.94" x2="71.12" y2="27.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="IO4"/>
<wire x1="71.12" y1="30.48" x2="78.74" y2="30.48" width="0.1524" layer="91"/>
<wire x1="78.74" y1="30.48" x2="78.74" y2="73.66" width="0.1524" layer="91"/>
<pinref part="X11" gate="G$1" pin="-2"/>
<wire x1="78.74" y1="73.66" x2="96.52" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="X11" gate="G$1" pin="-1"/>
<wire x1="96.52" y1="76.2" x2="83.82" y2="76.2" width="0.1524" layer="91"/>
<pinref part="X4" gate="G$1" pin="-3"/>
<wire x1="96.52" y1="15.24" x2="88.9" y2="15.24" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="3V3"/>
<wire x1="88.9" y1="15.24" x2="83.82" y2="15.24" width="0.1524" layer="91"/>
<wire x1="83.82" y1="15.24" x2="73.66" y2="15.24" width="0.1524" layer="91"/>
<wire x1="73.66" y1="15.24" x2="71.12" y2="15.24" width="0.1524" layer="91"/>
<wire x1="71.12" y1="15.24" x2="71.12" y2="20.32" width="0.1524" layer="91"/>
<pinref part="X5" gate="G$1" pin="-3"/>
<wire x1="96.52" y1="0" x2="73.66" y2="0" width="0.1524" layer="91"/>
<wire x1="73.66" y1="0" x2="73.66" y2="7.62" width="0.1524" layer="91"/>
<junction x="73.66" y="15.24"/>
<pinref part="X8" gate="G$1" pin="-3"/>
<wire x1="73.66" y1="7.62" x2="73.66" y2="15.24" width="0.1524" layer="91"/>
<wire x1="2.54" y1="7.62" x2="10.16" y2="7.62" width="0.1524" layer="91"/>
<junction x="73.66" y="7.62"/>
<pinref part="X7" gate="G$1" pin="-3"/>
<wire x1="10.16" y1="7.62" x2="73.66" y2="7.62" width="0.1524" layer="91"/>
<wire x1="2.54" y1="20.32" x2="10.16" y2="20.32" width="0.1524" layer="91"/>
<wire x1="10.16" y1="20.32" x2="10.16" y2="7.62" width="0.1524" layer="91"/>
<junction x="10.16" y="7.62"/>
<pinref part="X9" gate="G$1" pin="-3"/>
<wire x1="2.54" y1="-10.16" x2="10.16" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="10.16" y1="-10.16" x2="10.16" y2="7.62" width="0.1524" layer="91"/>
<pinref part="X2" gate="G$1" pin="-4"/>
<wire x1="96.52" y1="33.02" x2="88.9" y2="33.02" width="0.1524" layer="91"/>
<wire x1="88.9" y1="33.02" x2="88.9" y2="15.24" width="0.1524" layer="91"/>
<junction x="88.9" y="15.24"/>
<wire x1="83.82" y1="76.2" x2="83.82" y2="15.24" width="0.1524" layer="91"/>
<junction x="83.82" y="15.24"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
