#!/bin/bash

for K in ../result.registration/*.conf; do
	conf=$(basename $K)
	./registration_error.bin ../result.registration/${conf}/result ../../database/corrected/${conf} 2> ${conf}.error
done
