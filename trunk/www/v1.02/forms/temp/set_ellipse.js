<script language="JavaScript">
  function set_ellipse()
  {
  var myellipse = document.coordconv.ellipse_select.value;

  if( myellipse == "World Geodetic System 1984" )
  {
  document.coordconv.ellipse_code.value = 0;
  document.coordconv.ellipse_a.value = 6378137.0;
  document.coordconv.ellipse_f_inv.value = 298.257223563;
  document.coordconv.ellipse_b.value = 6356752.31424518;
  document.coordconv.ellipse_e2.value = 0.00669437999014132;
  }
  else if( myellipse == "Airy 1830" )
  {
  document.coordconv.ellipse_code.value = 1;
  document.coordconv.ellipse_a.value = 6377563.396;
  document.coordconv.ellipse_f_inv.value = 299.3249647;
  document.coordconv.ellipse_b.value = 6356256.9092444;
  document.coordconv.ellipse_e2.value = 0.00667053999776051;
  }
  else if( myellipse == "Modified Airy" )
  {
  document.coordconv.ellipse_code.value = 2;
  document.coordconv.ellipse_a.value = 6377340.189;
  document.coordconv.ellipse_f_inv.value = 299.3249647;
  document.coordconv.ellipse_b.value = 6356034.44794565;
  document.coordconv.ellipse_e2.value = 0.0066705399977606;
  }
  else if( myellipse == "Australian National" )
  {
  document.coordconv.ellipse_code.value = 3;
  document.coordconv.ellipse_a.value = 6378160.0;
  document.coordconv.ellipse_f_inv.value = 298.25;
  document.coordconv.ellipse_b.value = 6356774.7191953063;
  document.coordconv.ellipse_e2.value = 0.0066945418545876;
  }
  else if( myellipse == "Bessel 1841" )
  {
  document.coordconv.ellipse_code.value = 4;
  document.coordconv.ellipse_a.value = 6377397.155;
  document.coordconv.ellipse_f_inv.value = 299.1528128;
  document.coordconv.ellipse_b.value = 6356078.9628181886;
  document.coordconv.ellipse_e2.value = 0.0066945418545876;
  }
  else if( myellipse == "Clarke 1866" )
  {
  document.coordconv.ellipse_code.value = 5;
  document.coordconv.ellipse_a.value = 6378206.4;
  document.coordconv.ellipse_f_inv.value = 294.9786982;
  document.coordconv.ellipse_b.value = 6356583.7999989809;
  document.coordconv.ellipse_e2.value = 0.00676865799760959;
  }
  else if( myellipse == "Clarke 1880" )
  {
  document.coordconv.ellipse_code.value = 6;
  document.coordconv.ellipse_a.value = 6378249.145;
  document.coordconv.ellipse_f_inv.value = 293.465;
  document.coordconv.ellipse_b.value = 6356514.8695497755;
  document.coordconv.ellipse_e2.value = 0.00680351128284912;
  }
  else if( myellipse == "Everest(India 1830)" )
  {
  document.coordconv.ellipse_code.value = 7;
  document.coordconv.ellipse_a.value = 6377276.345;
  document.coordconv.ellipse_f_inv.value = 300.8017;
  document.coordconv.ellipse_b.value = 6356075.4131402392;
  document.coordconv.ellipse_e2.value = 0.00663784663019987;
  }
  else if( myellipse == "Everest(Brunei and E.Malaysia" )
  {
  document.coordconv.ellipse_code.value = 8;
  document.coordconv.ellipse_a.value = 6377298.556;
  document.coordconv.ellipse_f_inv.value = 300.8017;
  document.coordconv.ellipse_b.value = 6356097.5503008962;
  document.coordconv.ellipse_e2.value = 0.00663784663019965;
  }
  else if( myellipse == "Everest(W.Malaysia and Singapore" )
  {
  document.coordconv.ellipse_code.value = 9;
  document.coordconv.ellipse_a.value = 6377304.063;
  document.coordconv.ellipse_f_inv.value = 300.8017;
  document.coordconv.ellipse_b.value = 6356103.0389931547;
  document.coordconv.ellipse_e2.value = 0.00663784663019970;
  }
  else if( myellipse == "Geodetic Reference System 1980" )
  {
  document.coordconv.ellipse_code.value = 10;
  document.coordconv.ellipse_a.value = 6378137.0;
  document.coordconv.ellipse_f_inv.value = 298.257222101;
  document.coordconv.ellipse_b.value = 6356752.3141403561;
  document.coordconv.ellipse_e2.value = 0.00669438002290069;
  }
  else if( myellipse == "Helmert 1906" )
  {
  document.coordconv.ellipse_code.value = 11;
  document.coordconv.ellipse_a.value = 6378200.0;
  document.coordconv.ellipse_f_inv.value = 298.30;
  document.coordconv.ellipse_b.value = 6356818.1696278909;
  document.coordconv.ellipse_e2.value = 0.00669342162296610;
  }
  else if( myellipse == "Hough 1960" )
  {
  document.coordconv.ellipse_code.value = 12;
  document.coordconv.ellipse_a.value = 6378270.0;
  document.coordconv.ellipse_f_inv.value = 297.00;
  document.coordconv.ellipse_b.value = 6356794.3434343431;
  document.coordconv.ellipse_e2.value = 0.00672267002233347;
  }
  else if( myellipse == "International 1924" )
  {
  document.coordconv.ellipse_code.value = 13;
  document.coordconv.ellipse_a.value = 6378388.0;
  document.coordconv.ellipse_f_inv.value = 297.00;
  document.coordconv.ellipse_b.value = 6356911.9461279465;
  document.coordconv.ellipse_e2.value = 0.00672267002233323;
  }
  else if( myellipse == "South American 1969" )
  {
  document.coordconv.ellipse_code.value = 14;
  document.coordconv.ellipse_a.value = 6378160.0;
  document.coordconv.ellipse_f_inv.value = 298.25;
  document.coordconv.ellipse_b.value = 6356774.7191953063;
  document.coordconv.ellipse_e2.value = 0.00669454185458760;
  }
  else if( myellipse == "World Geodetic System 1972" )
  {
  document.coordconv.ellipse_code.value = 15;
  document.coordconv.ellipse_a.value = 6378135.0;
  document.coordconv.ellipse_f_inv.value = 298.26;
  document.coordconv.ellipse_b.value = 6356750.5200160937;
  document.coordconv.ellipse_e2.value = 0.00669431777826668;
  }
  document.coordconv_data.ellipse_code.value = document.coordconv.ellipse_code.value;  
  return true;
  }
</script>
