function ssm_out = fflib_ssm_removeSmallElements(ssm_in, d)
ssm_out = ssm_in;
ssm_out.a = fflib_removeSmallElements(ssm_out.a, d);
ssm_out.b = fflib_removeSmallElements(ssm_out.b, d);
ssm_out.c = fflib_removeSmallElements(ssm_out.c, d);
ssm_out.d = fflib_removeSmallElements(ssm_out.d, d);
end