function rt=rtau(dp)
if(dp<=25)
  rt=1.0;
elseif(dp>=50)
  rt=.1;
else
  rt=1.9-.036*dp;
end

