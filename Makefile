.PHONY: pkg

pkg:
	dpkg-buildpackage -us -uc -sd -b
rls:
	dch --distribution unstable -v `./version.sh` "Update debian pkg"
	dpkg-buildpackage -us -uc -sd -b
clean:
	debian/rules clean
